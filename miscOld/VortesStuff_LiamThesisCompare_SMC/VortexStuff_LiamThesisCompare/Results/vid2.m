clear all
close all
clc

vid = VideoWriter('yourvideo.avi');%create the video object
vid.FrameRate = 10;
open(vid); %open the file for writing

tsave = 0.01;
for ii=tsave:tsave:0.99 %where N is the number of images
   i = num2str(ii);
   picName = append("t=",i,"s");
   name = sprintf('%s%s', picName, '.png');
  I = imread(name); %read the next image
  writeVideo(vid,I); %write the image to file
end
close(vid);