Hey guys, to use the yolo detector, which runs automatically FOR THE INGITION LAUNCH ONLY now you need to do the following:

Install PIP

	sudo apt install python3-pip
Install deps

	# Remove the too-new version
	python3 -m pip uninstall -y numpy
	# Reinstall a version < 2.0 (safe with cv_bridge)
	python3 -m pip install "numpy<2.0" --user --upgrade

	pip3 install ultralytics 
	python3 -m pip install "opencv-python<=4.8.1.78" --user

	# for gpu (untested)
	pip3 install torch 
	pip3 install torchvision 
	pip3 install torchaudio 
	
	#for cpu
	python3 -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch torchvision torchaudio

