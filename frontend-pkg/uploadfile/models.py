from django.db import models

# Create your models here.
class QualityMeaser(models.Model):
    ipfs_hash = models.CharField(max_length=50)
    eth_address = models.CharField(max_length=50)
    timestamp = models.DateTimeField(auto_now_add=True)

    def __str__(self):
        return '%s %s %s %s' % (self.id, self.ipfs_hash, self.eth_address, self.timestamp)
