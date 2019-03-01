from django.shortcuts import render
from django.conf import settings as djangoSettings
from django.core.files.storage import FileSystemStorage
from django.shortcuts import redirect
from django.http import HttpResponse
from django.views.decorators.csrf import csrf_exempt

import time
import datetime
import pyqrcode
import zipfile
import os
import json
import rospy
import ipfsapi
import requests
from chemistry_services.srv import *

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

def extractConcentration(path):
    with open(path, encoding='cp1251') as f:    # not universal because of cp1252
        lines = f.read().splitlines()
        f.close()

        for l in lines:
            if l.startswith('Концентрация'):
                found = l.split(' ')[-1]
                break

        return found

@csrf_exempt
def index(request):
    if request.method == 'POST' and request.FILES['myfile']:
        timeStamp = getTimeStamp()
        infoDict = {}

        infoDict['date']            = timeStamp
        infoDict['batch_number']    = int(request.POST.get("batchNumber", ""))
        infoDict['place']           = request.POST.get("placeOfProduction", "")
        infoDict['owner']           = request.POST.get("technologyOwner", "")
        infoDict['selection']       = request.POST.get("responsibleForSelection", "")
        infoDict['responsible']     = request.POST.get("responsibleForBatch", "")
        infoDict['signature']       = request.POST.get("signature", "")

        myfile = request.FILES['myfile']
        # save file to local storage
        fs = FileSystemStorage()
        filename = fs.save(timeStamp + '/' + 'data', myfile)
        savePath = fs.path(filename)

        print(savePath)

        infoDict['concentration'] = extractConcentration(savePath)

        saveInfoPath = djangoSettings.MEDIA_ROOT + '/' + timeStamp + "/info.txt"
        print(saveInfoPath)
        with open(saveInfoPath, "w") as f:
            f.write(json.dumps(infoDict))
            f.close()

        zipFile = djangoSettings.MEDIA_ROOT + '/' + timeStamp + "/publish.zip"
        with zipfile.ZipFile(zipFile, 'w') as z:
            z.write(saveInfoPath, os.path.basename(saveInfoPath))
            z.write(savePath, os.path.basename(savePath))
            z.close()

        publishAFileToBC(zipFile)
        r = getResult()
        ipfsHash = r[0]
        ethAddress = r[1]

        shorturl = requests.post("https://url.today/yourls-api.php", data={'username': 'vadim.manaenko', 'password': '', 'action': 'shorturl', 'url': 'https://quality.nanodoctor.pro/getinfo/' + ipfsHash, 'format': 'simple'})

        # generate QR-code
        qrcode = pyqrcode.create(shorturl.text)
        print(djangoSettings.MEDIA_ROOT + '/' + timeStamp + '/' + 'qr.png')
        qrcode.png(djangoSettings.MEDIA_ROOT + '/' + timeStamp + '/' + 'qr.png', scale=5)

        uploaded_file_url = fs.url(timeStamp + '/qr.png')
        return redirect(uploaded_file_url)
    return render(request, 'uploadfile/index.html')

def getinfo(request, hash):
    print("hash is {}".format(hash))

    ipfs = ipfsapi.connect('127.0.0.1', 5001)
    os.chdir(djangoSettings.MEDIA_ROOT + '/')
    ipfs.get(hash)

    arguments = {}

    with zipfile.ZipFile(hash, 'r') as z:
        with z.open('info.txt', 'r') as info:
            data = info.read().decode('utf-8', 'ignore')
            arguments = json.loads(data)

    arguments['ipfs'] = hash

    '''
    arguments = {
        'date': row.timestamp,
        'batch_number': row.batch_number,
        'place': row.place,
        'owner': row.owner,
        'selection': row.responsible_for_selection,
        'responsible': row.responsible_for_batch,
        'concentration': row.concentration,
        'ipfs': row.ipfs_hash
    }
    '''

    return render(request, 'uploadfile/success.html', arguments)

