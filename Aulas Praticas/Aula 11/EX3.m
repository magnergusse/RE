clc
clear all

 serverIP = '192.168.0.100'; % IP address of remote host
 port = 9999;
 % Port to establish the communication
 t = tcpclient(serverIP, port, 'Timeout', 5); % create the tcp/ip lin
close all

img = vsGetImageFromServer(t);
imshow(img)

imwrite(img, 'imgobst.jpeg')
pause(0.01)

clear t

