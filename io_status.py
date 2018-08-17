# here we are gonna create a program to find out the io_status of the robot


import requests
import re
import time
from bs4 import BeautifulSoup
session = requests.Session()


try:
    a = session.get("http://192.168.125.1/rw/iosystem/signals/Input1",auth = requests.auth.HTTPDigestAuth('Default User','robotics'))
    print(a)

except:
    session =requests.Session()
    a = session.get("http://192.168.125.1/rw/iosystem/signals/Input1",auth = requests.auth.HTTPDigestAuth('Default User','robotics'))

ac = a.content
x = re.compile("<.+?>").findall(ac)
ac.replace(x[0],"")

dictval = {}
soup = BeautifulSoup(ac,'lxml')

for link in soup.find("li"):
    if not link:break
    title = link.get('title')
    dictval[title] = {}
    soup2 = BeautifulSoup(str(link))
    for llink in soup2.find_all('span'):
        dictval[title][llink.get('class')[0]] = llink.encode_contents()
    print title,dictval[title]


# this is the code to get the io status of the abb1200
#its gives the status of the input pin 1