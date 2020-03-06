.PHONY: build deploy toml

build:
	./gradlew --offline compileFrcUserProgramReleaseExecutableFrcUserProgramCpp

deploy:
	./gradlew deploy

toml:
	./gradlew deployFrcStaticFileDeployRoborio
