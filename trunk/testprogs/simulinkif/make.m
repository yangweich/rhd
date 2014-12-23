% Commands to make rhdlink s-function block
% NOTE: Run this file from Matlab

mex -v -g -I../../include ../../librhd/librhd.a rhdlink.cpp
