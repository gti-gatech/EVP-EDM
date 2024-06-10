# EVP-EDM

This repo contains a working example of an External Driver model in VISSIM.

The most important components are:

1) The Pullover Model Tutorial.pdf: 

This an indepth guide to modeling a custom driver behavior model in VISSIM using the External Driver Model (EDM)API.Section 8 of the tutorial explains the design process of a pullover model implementation in the proximtiy of Emergency Vehicles (ERVs). The C++ code and the required libraries have all been included in this repo. The instructions to compile and build the model has also been explained in the tutorial. 

2) DriverModel.dll (x64):

This is a completely assembled External Driver Model file which can be directly improted into any VISSIM model. This may only be used in systems on the x64 platform and any others would require a re-compile.

3) DriverModel.cpp

The C++ code for the EDM has been provided. This contains the Pullover Model example logic and algorithm.

4) DriverModel.zip

The zipped folder contains the entire codebase with all the libraries necessary for building the model on Visual Studio. The process is further explained in the tutorial

5) Interface_Description.pdf

This is an official technical manual for EDMs published by PTV. Cross referecning this with our tutorial would be the ideal way to efficiiently build a custom model.


