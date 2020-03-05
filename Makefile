.PHONY: build deploy toml

build:
	./gradlew --offline compile

deploy:
	./gradlew deploy

toml:
	./gradlew deployFrcStaticFileDeployRoborio
