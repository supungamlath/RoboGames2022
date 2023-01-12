.ONESHELL:

init:
	sudo apt-get install qtwayland5 python3-opencv
	pip install -r requirements.txt
	mkdir logs

installwebots:
	wget https://github.com/cyberbotics/webots/releases/download/R2022b/webots_2022b_amd64.deb
	sudo apt install --no-upgrade ./webots_2022b_amd64.deb

test:
	python3 -m pytest

run:
	export WEBOTS_HOME="/usr/local/webots"
	export PYTHONPATH="$$WEBOTS_HOME/lib/controller/python310:$$PATH"
	export PYTHONPATH="$$WEBOTS_HOME/lib/controller/python39:$$PATH"
	python3 dave_controller.py & webots