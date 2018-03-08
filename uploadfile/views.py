from django.shortcuts import render
from django.conf import settings as djangoSettings
from django.core.files.storage import FileSystemStorage

import time
import datetime

def getTimeStamp():
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
    return st

def index(request):
    if request.method == 'POST' and request.FILES['myfile']:
        myfile = request.FILES['myfile']
        fs = FileSystemStorage()
        savePath = djangoSettings.STATIC_ROOT + '/' + getTimeStamp() + '/' + myfile.name
        filename = fs.save(savePath, myfile)
        uploaded_file_url = fs.url(filename)
        return render(request, 'uploadfile/index.html', {
            'uploaded_file_url': uploaded_file_url
        })
    return render(request, 'uploadfile/index.html')

