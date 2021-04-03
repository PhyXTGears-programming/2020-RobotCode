#include "commands/FollowPolybezier.h"

#include <fstream>
#include <tuple>

#include <wpi/raw_istream.h>
#include <frc/RobotController.h>

constexpr double PI = 3.1415926535897932;

FollowPolybezier::FollowPolybezier (Drivetrain* drivetrain, const wpi::Twine &filename, Configuration configuration) :
    drivetrain(drivetrain), config(configuration)
{
    AddRequirements(drivetrain);

    std::error_code code;
    wpi::raw_fd_istream pathFile {filename, code};

    if (code.value() != 0) {
        std::cerr << "Unable to open file \"" << filename.str() << "\"" << std::endl;
        std::cerr << code.message() << std::endl;
        throw ENOENT;
    }

    wpi::json pathJSON;
    pathFile >> pathJSON;

    for (auto val : pathJSON) {
        LoadCurve(val);
    }
}

void FollowPolybezier::Initialize () {
    logFile = new std::ofstream("/home/lvuser/logNew1.txt");

    finished = false;

    if (polybezier.size() < 1) { // there must be at least one curve
        finished = true;
        Cancel();
        return;
    }

    currentBezier = 0;
    ResetCurveProgress();

    drivetrain->SetAcceleration(0, 0);
    velocity = 0;
    acceleration = 0;
    lastTime = frc::RobotController::GetFPGATime();
}

void FollowPolybezier::Execute () {
    auto pose = drivetrain->GetPose();
    distanceTraveled += pose.Translation().Distance(lastPose.Translation()).to<double>();

    // std::cout << pose.Translation().X().to<double>()
    //     << ", " << pose.Translation().Y().to<double>()
    //     << ", " << pose.Rotation().Degrees().to<double>()
    //     << std::endl;

    auto *currentStep = &polybezier[currentBezier];

    while (distanceTraveled >= currentStep->second[prevVertex+1].d) {
        prevVertex++;
        if (prevVertex + 1 >= currentStep->second.size()) { // if new prevVertex is the last
            currentBezier++;
            if (currentBezier >= polybezier.size()) {
                finished = true;
                return;
            }
            ResetCurveProgress();
        }
    }

    double dT = currentStep->second[prevVertex+1].t - currentStep->second[prevVertex].t; // total change in t over the current segment
    double dD = currentStep->second[prevVertex+1].d - currentStep->second[prevVertex].d; // total change in d over the current segment

    double distanceAlongSegment = distanceTraveled - currentStep->second[prevVertex].d; // change in d since the beginning of the segment
    double timeAlongSegment = distanceAlongSegment * (dT/dD); // approximate change in t since the beginning of the segment

    double t = currentStep->second[prevVertex].t + timeAlongSegment; // approximate t value for the current position (initial + change)

    auto derivs = Bezier::evaluateDerivatives(currentStep->first, t);
    double r = Bezier::getRadiusOfCurvature(derivs);

    double v = drivetrain->GetSpeed();
    double w = v/r; // get the angular velocity needed to drive in a circle of radius r at velocity v

    // get the current target angle using the tangent at the current point
    double angle = std::atan2(derivs.firstDeriv.y, derivs.firstDeriv.x);

    // shift by 360 degrees if one of the values rolls over but the other doesn't
    double robotAngle = pose.Rotation().Radians().to<double>();
    if (robotAngle - angle > 4.5) {
        angle += 2*PI;
    } else if (angle - robotAngle > 4.5) {
        angle -= 2*PI;
    }

    std::cout << frc::RobotController::GetFPGATime() << "," << angle << "," << robotAngle << std::endl;

    auto accel = CalculateAcceleration();

    drivetrain->SetAcceleration(accel.first, accel.second);
    drivetrain->SetAngularVelocity(w, angle);

    lastPose = pose;
}

std::pair<double, double> FollowPolybezier::CalculateAcceleration () {
    // get time since last execution
    uint64_t currentTime = frc::RobotController::GetFPGATime();
    double dt = (currentTime - lastTime) / 1'000'000.0;
    lastTime = currentTime;

    // compute velocity change since last execution
    velocity += acceleration * dt;

    double cVelocity = velocity;
    double cAcceleration = acceleration;
    unsigned int cBezier = currentBezier;
    unsigned int cVertex = prevVertex;

    std::pair<double, std::pair<unsigned int, unsigned int>> leastMargin = {cVelocity+1, {0, 0}};

    std::vector<std::tuple<std::pair<unsigned int, unsigned int>, double, double, double>> points;

    while (cVelocity > 0) {
        cVertex++;
        if (cVertex >= polybezier[cBezier].second.size()) {
            cVertex = 0;
            cBezier++;
            if (cBezier >= polybezier.size()) {
                if (-cVelocity < leastMargin.first) leastMargin = {-cVelocity, {cBezier-1, polybezier[cBezier].second.size()-1}};
                break;
            }
        }

        auto *currentApproximation = &polybezier[cBezier].second;

        double distTraveled;
        if (cVertex == 0) {
            distTraveled = 0;
        } else if (cBezier == currentBezier && cVertex == prevVertex+1) {
            distTraveled = (*currentApproximation)[cVertex].d - distanceTraveled;
        } else {
            distTraveled = (*currentApproximation)[cVertex].d - (*currentApproximation)[cVertex-1].d;
        }

        // compute change in acceleration and velocity over distTraveled
        double vSquared = cVelocity*cVelocity + 2*cAcceleration*distTraveled;
        double newVelocity = vSquared > 0 ? std::sqrt(vSquared) : 0;
        double dt = (newVelocity - cVelocity) / cAcceleration;
        cAcceleration -= config.maximumJerk * dt;
        cAcceleration = cAcceleration < -config.maximumReverseAcceleration ? -config.maximumReverseAcceleration : cAcceleration;
        cVelocity = newVelocity;

        points.push_back({{cBezier, cVertex}, distTraveled, cVelocity, cAcceleration});

        *logFile << "@ " << cBezier << "," << cVertex << "," << polybezier[cBezier].second[cVertex].maxV << ", " << cVelocity << ", " << cAcceleration << std::endl;

        double margin = polybezier[cBezier].second[cVertex].maxV - cVelocity;
        if (margin < leastMargin.first) leastMargin = {margin, {cBezier, cVertex}};
    }

    // limit based on point past min
    if (leastMargin.second.first > nextMin.first || (leastMargin.second.first == nextMin.first && leastMargin.second.second > nextMin.second)) {
        // act like normal
    } else {
        // calculate margin for min:
        // get min distance
        auto minPoint = polybezier[nextMin.first].second[nextMin.second];
        double mDistance = minPoint.d; // add distance from beginning of bezier with nextMin to nextMin
        mDistance -= polybezier[cBezier].second[cVertex].d; // remove distance from beginning of current bezier to current point

        if (cBezier < nextMin.first) { // if nextMin is in a later bezier curve
            int nB = cBezier-1;
            while (++nB < nextMin.first) {
                mDistance += polybezier[nB].second.end()->d;
            }
        }

        // find d
        double v = velocity;
        double q1 = (1.0/3.0)/(config.maximumJerk*config.maximumJerk) * acceleration*acceleration;
        double q2 = (1.0/config.maximumJerk)*v;
        double d = (q1-q2) * acceleration;

        // check if we need to start reducing acceleration
        if (mDistance < d + 0.05 && acceleration < 0) {
            // bring acceleration to 0
            leastMargin.first = 0.2; // override margin
        } else {
            // calculate margin and adjust leastMargin appropriately
            double tD = 0;
            int index = -2;
            for (int i = 0; i < points.size(); i++) {
                tD += std::get<1>(points[i]);
                double v = std::get<2>(points[i]);
                double a = std::get<3>(points[i]);

                double q1 = (1.0/3.0)/(config.maximumJerk*config.maximumJerk) * a*a;
                double q2 = (1.0/config.maximumJerk)*v;
                double d = (q1-q2) * a;

                if (mDistance - tD < d) {
                    index = i-1;
                    break;
                }
            }
            if (index > -2) { 
                if (index == -1) index = 0;
                double v = std::get<2>(points[index]);
                double a = std::get<3>(points[index]);
                double t = -a/config.maximumJerk;
                double dv = (a + t*config.maximumJerk/2)*t;
                v += dv;
                double margin = minPoint.maxV - v;
                if (margin < leastMargin.first) leastMargin.first = margin;
            }
        }
    }

    // set acceleration
    if (leastMargin.first < 0.05) {
        acceleration -= config.maximumJerk*dt;
    } else if (leastMargin.first < 0.1) {
        // do nothing
    } else {
        acceleration += config.maximumJerk*dt;
    }

    acceleration = std::clamp(acceleration, -config.maximumReverseAcceleration, std::min(drivetrain->GetMaxAvailableAcceleration(), 1.0));

    double targetVelocity = velocity + 0.5*acceleration*dt;

    *logFile << currentTime << ", " << dt << ", " << acceleration << ", " << velocity << ", " << leastMargin.first << std::endl;

    return {acceleration, targetVelocity};
}

void FollowPolybezier::LoadCurve (wpi::json::value_type controlPoints) {
    std::pair<Bezier::CubicBezier, std::vector<FollowPolybezier::DistanceSample>> result {{
        {controlPoints[0][0], controlPoints[0][1]},
        {controlPoints[1][0], controlPoints[1][1]},
        {controlPoints[2][0], controlPoints[2][1]},
        {controlPoints[3][0], controlPoints[3][1]}
    }, {}};

    double l = 0;
    if (polybezier.size() > 0) {
        l = polybezier[polybezier.size()-1].second.end()->maxV;
    }

    AddApproximation(&result, l);

    polybezier.push_back(result);
}

void FollowPolybezier::AddApproximation (std::pair<Bezier::CubicBezier, std::vector<DistanceSample>> *bezier, double previousCurveFinalSpeed) {
    auto samples = Bezier::polylineApproximation(bezier->first, 1.001, 0.05);
    int nSamples = samples.size();

    bezier->second.clear();
    bezier->second.reserve(nSamples);

    // std::cout << "curve:" << std::endl;

    bezier->second.push_back({samples[0].p, samples[0].t, 0, 100, false});

    double distance = 0;
    for (int i = 1; i < nSamples; i++) {
        distance += Point::distance(samples[i-1].p, samples[i].p);

        double r = Bezier::getRadiusOfCurvature(bezier->first, samples[i].t);
        double maxV = std::sqrt(config.maximumRadialAcceleration * std::fabs(r));

        if (i == nSamples-1) maxV = 100;

        bool lessPrev = bezier->second[i-1].maxV < (i>1 ? bezier->second[i-2].maxV : previousCurveFinalSpeed);
        bool lessThis = bezier->second[i-1].maxV < maxV;
        if (lessPrev && lessThis) {
            bezier->second[i-1].minimum = true;
            // std::cout << "minimum: " << i-1 << "," << bezier->second[i-1].t << "," << bezier->second[i-1].maxV << std::endl;
        }

        // std::cout << i-1 << "," << samples[i-1].p.x << "," << samples[i-1].p.y << "," << maxV << "," << (bezier->second[i-1].minimum ? 1 : 0) << std::endl;

        bezier->second.push_back({samples[i].p, samples[i].t, distance, maxV, false});
    }

    // std::cout << nSamples-1 << "," << samples[nSamples-1].p.x << "," << samples[nSamples-1].p.y << "," << (*bezier->second.end()).maxV << ",";
    // std::cout << ((*bezier->second.end()).minimum ? 1 : 0) << std::endl;
}

void FollowPolybezier::SetNextMin () {
    unsigned int cBezier = currentBezier;
    unsigned int cVertex = prevVertex;

    while (true) {
        if (cVertex >= polybezier[cBezier].second.size()) {
            cVertex = 0;
            cBezier++;
            if (cBezier >= polybezier.size()) {
                nextMin = {cBezier,cVertex};
                break;
            }
        }
        if (polybezier[cBezier].second[cVertex].minimum) {
            nextMin = {cBezier,cVertex};
            break;
        }
        cVertex++;
    }
}

void FollowPolybezier::ResetCurveProgress () {
    // reset the variables that track progress along the curve
    prevVertex = 0;
    distanceTraveled = 0;

    // start tracking distance along the new Bezier curve from the current position
    lastPose = drivetrain->GetPose();

    // correct for the real current position
    polybezier[currentBezier].first.p0 = {lastPose.X().to<double>(), lastPose.Y().to<double>()};
    AddApproximation(&(polybezier[currentBezier]));

    SetNextMin();
}
