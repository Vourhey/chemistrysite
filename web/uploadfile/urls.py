from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path('getinfo/<int:id>', views.getinfo, name='getinfo'),
]

