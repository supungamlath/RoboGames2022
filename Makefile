init:
	pip install -r requirements.txt

test:
	python -m pytest

run:
	python dave_controller.py