from django.shortcuts import render
from django.conf import settings as djangoSettings
from django.core.files.storage import FileSystemStorage
from django.shortcuts import redirect
from django.http import HttpResponse
from django.views.decorators.csrf import csrf_exempt

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

def getResult():
    rospy.wait_for_service("/get_result")
    try:
        pub = rospy.ServiceProxy('/get_result', PublishToBC)
        r = pub("")
        return [r.result, r.address]            
    except rospy.ServiceException as e:
        print ("Service call failed: {}".format(e))

def publishAFileToBC(path):
    rospy.wait_for_service("/file_for_publishing")
    try:
        pub = rospy.ServiceProxy("/file_for_publishing", PublishToBC)
        r = pub(path)
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))

@csrf_exempt
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

        publishAFileToBC(savePath)
        r = getResult()
        ipfsHash = r[0]
        ethAddress = r[1]

        # save to DB
        row = QualityMeaser.objects.create(ipfs_hash=ipfsHash, eth_address=ethAddress)
        row.save()

        # generate QR-code
        qrcode = pyqrcode.create('https://quality.nanodoctor.pro/getinfo/' + str(row.id))
        print(djangoSettings.MEDIA_ROOT + '/' + timeStamp + '/' + 'qr.png')
        qrcode.png(djangoSettings.MEDIA_ROOT + '/' + timeStamp + '/' + 'qr.png', scale=5)

        uploaded_file_url = fs.url(timeStamp + '/qr.png')
        return redirect(uploaded_file_url)
    return render(request, 'uploadfile/index.html')

def getinfo(request, id):
    print("id is {}".format(id))
    '''
    row = QualityMeaser.objects.get(id=id)
    print(row.ipfs_hash)
    
    api = ipfsapi.connect('127.0.0.1', 5001)
    content = api.cat(row.ipfs_hash)
    content = content.decode(errors="replace")
    '''
    return render(request, 'uploadfile/success.html')

