from django.db import models

# Create your models here.
class QualityMeaser(models.Model):
    path_to_file = models.CharField(max_length=200)
    ipfs_hash = models.CharField(max_length=50)
    eth_address = models.CharField(max_length=50)

    def __str__():
        return '%s %s %s %s' % (self.id, self.path_to_file, self.ipfs_hash, self.eth_address)
