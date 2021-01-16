function TransmissionEvaluation()
% FUNCTION NAME: 
%   TransmissionEvaluation()
%
% DESCRIPTION:
%   Computes the the average intensity of all binary 
%	images in a directory selected by the user.
%
% INPUT:
%   None
%
% OUTPUT:
%   None
%
% Created:
%   Author:		Lukas Schwoerer
%	Date:		03.07.2020
%	Version:	V1.0
%

%% Initialize variables
clear all
listcounter = 1;


%% Select image folder and compile image list
path = uigetdir(pwd, 'Select image folder');
dircontent = dir(path);

for i = 1 : length(dircontent)	
	if contains(dircontent(i).name, '.bin')
		
		imagelist(listcounter) = strcat(dircontent(i).folder, "/", dircontent(i).name);
		listcounter = listcounter + 1;
		
	end	
end


%% Calculate mean value for all images in imagelist
for i = 1 : length(imagelist)
	
	fid = fopen(imagelist(i),'r');
    tmpimg = fread(fid,[2048, 2048],'*uint16'); %Read images from binary file
    fclose(fid);
	
	tmpimg = double(tmpimg)/2^12; %Scale 16bit image value into a range from 0-1
	
	disp(imagelist(i)); %Display image name
	disp(mean(tmpimg, 'all')); %Display mean intensity
	
end
end
