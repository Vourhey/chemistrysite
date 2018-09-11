from django.urls import path, re_path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    re_path(r'^getinfo/(?P<hash>Qm[a-zA-Z0-9]+$)', views.getinfo, name='getinfo'),
]

