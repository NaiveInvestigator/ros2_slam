.PHONY: install docker

install:
	@pip install --verbose .

uninstall:
	@pip -v uninstall orb_slam3

editable:
	@pip install scikit-build-core pyproject_metadata pathspec pybind11 ninja cmake
	@pip install --no-build-isolation -ve .

docker:
	docker build -t orb_ubuntu24 -f docker/Dockerfile.ubuntu24 .

ubuntu22:
	docker build -t orb_ubuntu22 -f docker/Dockerfile.ubuntu22 .

run:
	docker run -it --rm orb_ubuntu24 /bin/bash

run22:
	docker run -it --rm orb_ubuntu22 /bin/bash
