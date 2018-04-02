from django.shortcuts import render
from django.conf import settings as djangoSettings
from django.core.files.storage import FileSystemStorage
from django.shortcuts import redirect

import time
import datetime
import ipfsapi
import pyqrcode
import os
from .models import QualityMeaser

def getTimeStamp():
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
    return st

def publishToIPFS(path):
    api = ipfsapi.connect('127.0.0.1', 5001)
    res = api.add(path)
    return res['Hash']

def index(request):
    if request.method == 'POST' and request.FILES['myfile']:
        myfile = request.FILES['myfile']

        # save file to local storage
        fs = FileSystemStorage()
        filename = fs.save(getTimeStamp() + '/' + myfile.name, myfile)
        savePath = fs.url(filename)

        print(myfile.name)
        print(filename)
        print(savePath)

        # publish to IPFS
        ipfsHash = publishToIPFS(djangoSettings.MEDIA_ROOT + '/' + filename)
        #ipfsHash = "Qm"

        ethAddress = "0x" #callPublishnode(ipfsHash)

        # save to DB
        row = QualityMeaser.objects.create(path_to_file=savePath, ipfs_hash=ipfsHash, eth_address=ethAddress)
        row.save()

        # generate QR-code
        qrcode = pyqrcode.create("http://ipfs.io/ipfs/" + ipfsHash)
        print(djangoSettings.MEDIA_ROOT + '/' + getTimeStamp() + '/' + 'qr.png')
        qrcode.png(djangoSettings.MEDIA_ROOT + '/' + getTimeStamp() + '/' + 'qr.png', scale=5)

        uploaded_file_url = fs.url(getTimeStamp() + '/qr.png')
        '''      return render(request, 'uploadfile/index.html', {
            'uploaded_file_url': uploaded_file_url
        })'''
        return redirect(uploaded_file_url)
    return render(request, 'uploadfile/index.html')

