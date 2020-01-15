# only run the following line to generate the preliminary rst files
# you don't need to run this line in general
# this line is used when the docs are first created
sphinx-apidoc -f --separate -o source/tmp ../src/airobot

# make sure the changes in the source code are reflected in the docs
pip install ..
# build docs
make html
