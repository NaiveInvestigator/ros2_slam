.PHONY:ubuntu24 ubuntu22
ubuntu24:
	docker build -t orb_ubuntu24 -f Dockerfile.ubuntu24 .

ubuntu22:
	docker build -t orb_ubuntu22 -f Dockerfile.ubuntu24 .

run:
	docker run -it --rm orb_ubuntu24 /bin/bash

run22:
	docker run -it --rm orb_ubuntu22 /bin/bash
