from django.shortcuts import render
from django.conf import settings as djangoSettings
from django.core.files.storage import FileSystemStorage
from django.shortcuts import redirect
from django.http import HttpResponse

import time
import datetime
import pyqrcode
import os
import rospy
import ipfsapi
from chemistry_services.srv import * 
from .models import QualityMeaser

def getTimeStamp():
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
    return st

def callPublishToBC(path):
    rospy.wait_for_service("publish_to_bc")
    try:
        pub = rospy.ServiceProxy('publish_to_bc', PublishToBC)
        r = pub(path)
        return [r.result, r.address]            
    except rospy.ServiceException as e:
        print ("Service call failed: {}".format(e))

def index(request):
    if request.method == 'POST' and request.FILES['myfile']:
        myfile = request.FILES['myfile']
        timeStamp = getTimeStamp()

        # save file to local storage
        fs = FileSystemStorage()
        filename = fs.save(timeStamp + '/' + myfile.name, myfile)
        savePath = fs.path(filename)

        print(myfile.name)
        print(filename)
        print(savePath)

        r = callPublishToBC(savePath)
        ipfsHash = r[0]
        ethAddress = r[1]

        # save to DB
        row = QualityMeaser.objects.create(ipfs_hash=ipfsHash, eth_address=ethAddress)
        row.save()

        # generate QR-code
        qrcode = pyqrcode.create('http://aira-csisensor.westeurope.cloudapp.azure.com:2345/getinfo/' + str(row.id))
        print(djangoSettings.MEDIA_ROOT + '/' + timeStamp + '/' + 'qr.png')
        qrcode.png(djangoSettings.MEDIA_ROOT + '/' + timeStamp + '/' + 'qr.png', scale=5)

        uploaded_file_url = fs.url(timeStamp + '/qr.png')
        '''      return render(request, 'uploadfile/index.html', {
            'uploaded_file_url': uploaded_file_url
        })'''
        return redirect(uploaded_file_url)
    return render(request, 'uploadfile/index.html')

def getinfo(request, id):
    print("id is {}".format(id))
    row = QualityMeaser.objects.get(id=id)
    print(row.ipfs_hash)
    
    api = ipfsapi.connect('127.0.0.1', 5001)
    content = api.cat(row.ipfs_hash)
    content = content.decode(errors="replace")
    return HttpResponse('<pre>' + content + '</pre>')

