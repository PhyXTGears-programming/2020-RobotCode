#include "commands/FollowPolybezier.h"

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
        polybezier.push_back(LoadCurve(val));
    }
}

void FollowPolybezier::Initialize () {
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

    // shift the angle (-pi to pi) by multiples of 2pi to get the correct angle relative to the robot angle (-infinity to infinity)
    angle += std::floor((pose.Rotation().Radians().to<double>() + PI) / (2*PI)) * 2*PI;

    auto accel = CalculateAcceleration();

    // std::cout << "acceleration = " << accel.first << ", velocity = " << accel.second << ", angular velocity = " << w << std::endl;

    auto pathPose = Bezier::evaluate(currentStep->first, t);
    std::cout
        << pathPose.x << "," << pathPose.y << "," << angle
        << "," << pose.Translation().X().to<double>() << "," << pose.Translation().Y().to<double>()
        << "," << pose.Rotation().Degrees().to<double>() << std::endl;

    drivetrain->SetAcceleration(accel.first, accel.second);
    drivetrain->SetAngularVelocity(w, angle);

    lastPose = pose;
}

std::pair<double, double> FollowPolybezier::CalculateAcceleration () {
    double cVelocity = drivetrain->GetSpeed();
    double cAcceleration = acceleration;
    unsigned int cBezier = currentBezier;
    unsigned int cVertex = prevVertex;

    double leastMargin = cVelocity+1;

    while (cVelocity > 0) {
        cVertex++;
        if (cVertex >= polybezier[cBezier].second.size()) {
            cVertex = 0;
            cBezier++;
            if (cBezier >= polybezier.size()) {
                if (-cVelocity < leastMargin) leastMargin = -cVelocity;
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

        // std::cout << polybezier[cBezier].second[cVertex].maxV << ", " << cVelocity << ", " << cAcceleration << std::endl; 

        double margin = polybezier[cBezier].second[cVertex].maxV - cVelocity;
        if (margin < leastMargin) leastMargin = margin;
    }

    uint64_t currentTime = frc::RobotController::GetFPGATime();
    double dt = (currentTime - lastTime) / 1'000'000.0;
    lastTime = currentTime;

    velocity += acceleration * dt;

    if (leastMargin < 0.05) {
        acceleration -= config.maximumJerk*dt;
    } else if (leastMargin < 0.1) {
        // do nothing
    } else {
        acceleration += config.maximumJerk*dt;
    }

    acceleration = std::clamp(acceleration, -config.maximumReverseAcceleration, drivetrain->GetMaxAvailableAcceleration());

    double targetVelocity = velocity + 0.5*acceleration*dt;

    // std::cout << currentTime << ", " << dt << ", " << acceleration << ", " << velocity << ", " << leastMargin << std::endl;

    return {acceleration, targetVelocity};
}

std::pair<Bezier::CubicBezier, std::vector<FollowPolybezier::DistanceSample>> FollowPolybezier::LoadCurve (wpi::json::value_type controlPoints) {
    std::pair<Bezier::CubicBezier, std::vector<FollowPolybezier::DistanceSample>> result {{
        {controlPoints[0][0], controlPoints[0][1]},
        {controlPoints[1][0], controlPoints[1][1]},
        {controlPoints[2][0], controlPoints[2][1]},
        {controlPoints[3][0], controlPoints[3][1]}
    }, {}};

    AddApproximation(&result);

    return result;
}

void FollowPolybezier::AddApproximation (std::pair<Bezier::CubicBezier, std::vector<DistanceSample>> *bezier) {
    auto samples = Bezier::polylineApproximation(bezier->first);

    bezier->second.clear();
    bezier->second.reserve(samples.size());


    // std::cout << "curve:" << std::endl;

    Point::Point previous = samples[0].p;
    double distance = 0;
    for (auto point : samples) {
        distance += Point::distance(previous, point.p);

        double r = Bezier::getRadiusOfCurvature(bezier->first, point.t);
        double maxV = std::sqrt(config.maximumRadialAcceleration * std::fabs(r));

        // std::cout << point.p.x << "," << point.p.y << "," << r << "," << std::fabs(r) << "," << maxV << std::endl;

        bezier->second.push_back({point.p, point.t, distance, maxV});

        previous = point.p;
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
}
