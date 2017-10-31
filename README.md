# WebServer_MCube_On_MKR1000_Ajax
Webserver sending MCube 3672 x,y,z accelerations
This folder contains an Arduino sketch meant to be used on Arduino MKR1000.
The Arduino sketch provides a generic basic web server that produces a 
  webpage that displays values of some 3 exemplary variables called X,Y,Z. 
  It allows AJAX CORS. To use it:
  1- Enter the SSID and PWD below for your network.
  2- Upload to the MKR1000.
  3- Find the IP of this MKR1000 via the Serial Monitor. If the
     WiFi connection is successful the IP should be printed in 
     the Serial Monitor in few seconds.
  4- Go to a browser on a computer on the same network, and enter any of these
     URLs (REST style):
     http://MKR1000IPAddress/XYZ 
     http://MKR1000IPAddress/X
     http://MKR1000IPAddress/Y
     http://MKR1000IPAddress/Z    
  One can change the server to accept different URLs and serve other variables
  and different web pages. The variables would be measurements from the Arduino
  sensors.
  S.M.
  
  Also this folder contains a web page to fetch and display the data using AJAX.
  Make sure you enter the IP address of the MKR1000 on the webpage.
  
  S.M.
  
