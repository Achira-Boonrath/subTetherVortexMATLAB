function [outputArg1,outputArg2] = untitled2(inputArg1,inputArg2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    function sub1
        inputArg1 = 3;
        kk = 4;
    end

    function sub2
        inputArg2 = 3;
        inputArg1 = 4;
    end
sub1
% sub2
outputArg1 = inputArg1;
outputArg2 = inputArg2;

end