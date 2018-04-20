from django.shortcuts import render
from django.conf import settings as djangoSettings
from django.core.files.storage import FileSystemStorage
from django.shortcuts import redirect

import time
import datetime
import pyqrcode
import os
import rospy
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
        r = pub(savePath)
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
        savePath = fs.url(filename)

        print(myfile.name)
        print(filename)
        print(savePath)

        r = callPublishToBC(savePath)
        ipfsHash = r.result
        ethAddress = r.address

        # save to DB
        row = QualityMeaser.objects.create(ipfs_hash=ipfsHash, eth_address=ethAddress, timestamp=timeStamp)
        row.save()

        # generate QR-code
        qrcode = pyqrcode.create(row.id)
        print(djangoSettings.MEDIA_ROOT + '/' + timeStamp + '/' + 'qr.png')
        qrcode.png(djangoSettings.MEDIA_ROOT + '/' + timeStamp + '/' + 'qr.png', scale=5)

        uploaded_file_url = fs.url(timestamp + '/qr.png')
        '''      return render(request, 'uploadfile/index.html', {
            'uploaded_file_url': uploaded_file_url
        })'''
        return redirect(uploaded_file_url)
    return render(request, 'uploadfile/index.html')

