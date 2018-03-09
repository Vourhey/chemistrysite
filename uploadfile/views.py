from django.shortcuts import render
from django.conf import settings as djangoSettings
from django.core.files.storage import FileSystemStorage

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
        savePath = djangoSettings.STATIC_ROOT + '/' + getTimeStamp() + '/' + myfile.name
        filename = fs.save(savePath, myfile)

        # publish to IPFS
        ipfsHash = publishToIPFS(savePath)

        # save to DB
        row = QualityMeaser.objects.create(path_to_file=savePath, ipfs_hash=ipfsHash)
        row.save()

        # generate QR-code
        qrcode = pyqrcode.create(ipfsHash)
        qrcode.png(os.path.dirname(savePath) + '/' + 'qr.png', scale=5)

        uploaded_file_url = os.path.dirname(savePath) + '/' + 'qr.png'
        return render(request, 'uploadfile/index.html', {
            'uploaded_file_url': uploaded_file_url
        })
    return render(request, 'uploadfile/index.html')

