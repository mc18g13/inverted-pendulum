IMAGE_NAME = rotary-inverted-pendulum-pico-build
IMAGE_VERSION = latest
IMAGE_USER = mc18g13
IMAGE_TAG = $(IMAGE_USER)/$(IMAGE_NAME):$(IMAGE_VERSION)

WORKING_DIR := $(shell pwd)
DOCKER_DIR :=$(IMAGE_NAME)

UID = $(shell id -u)
GID = $(shell id -g)

.DEFAULT_GOAL := build-full

check-docker:: ## Target to check if docker is installed
	$(if $(shell which docker),,$(error "Docker needs to be installed to run this makefile"))

docker-build:: check-docker ## Build the docker image
	@echo Building $(IMAGE_TAG)
	@docker build \
		-t $(IMAGE_TAG) $(WORKING_DIR) \
		--build-arg USER=${USER} \
		--build-arg HOME=${HOME} \
		--build-arg UID=${UID} \
		--build-arg GID=${GID}

docker-push:: check-docker ## push the docker image
	@echo Pushing $(IMAGE_TAG)
	@docker push $(IMAGE_TAG)

DOCKER_BUILD_COMMAND = 	@docker run -t \
	-v $(WORKING_DIR):/$(DOCKER_DIR) \
	--user "${UID}:${GID}" \
	-w /$(DOCKER_DIR) $(IMAGE_TAG) bash -C \
	./scripts/build.sh --color=auto

build:: ## Build the firmare
	${DOCKER_BUILD_COMMAND}

docker-debug:: ## Enter the docker container to debug
	@echo ${DOCKER_BUILD_COMMAND}

build-full: docker-build build ## Setup the docker image and build the firmware

bu: build upload

upload:
	./scripts/upload.sh

clean:
	rm -rf cmake-build

usb:
	./scripts/usb_output.sh

# A help target including self-documenting targets (see the awk statement)
define HELP_TEXT
Usage: make [TARGET]... [MAKEVAR1=SOMETHING]...

Available targets:
endef
export HELP_TEXT
help: ## This help target
	@echo
	@echo "$$HELP_TEXT"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / \
		{printf "\033[36m%-30s\033[0m  %s\n", $$1, $$2}' $(MAKEFILE_LIST)