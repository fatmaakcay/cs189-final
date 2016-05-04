try:
    import json
except ImportError:
    import simplejson as json

from twitter import Twitter, OAuth, TwitterHTTPError, TwitterStream
import random

phrases = ["They don't want you to win", "The key to success is good pictures", "Bless up", "Liooon", "They never said winning was easy"]

# Variables that contains the user credentials to access Twitter API 
ACCESS_TOKEN = '723567770995200000-qTCtogXKBsURHBzKRMOtWsTpazbJ8k2'
ACCESS_SECRET = 'hW6l8NzvdOwYTEJowSKE2mbaGq469Y4cZrc7kAh1SZIHU'
CONSUMER_KEY = 'ThBUHLtWwQXs9yvvVM4C6yLT3'
CONSUMER_SECRET = 'Bc3D1XlLVB5W27KsgktO2rv0Uljqgw7WdfLELHToAtzEvdFVz9'

def post_twitter(take_three):


	t = Twitter(
	    auth=OAuth(ACCESS_TOKEN, ACCESS_SECRET, CONSUMER_KEY, CONSUMER_SECRET))



	with open("image1.png", "rb") as imagefile1:
	    imagedata1 = imagefile1.read()

	with open("image2.png", "rb") as imagefile2:
	    imagedata2 = imagefile2.read()

	with open("image3.png", "rb") as imagefile3:
	    imagedata3 = imagefile3.read()

	t_up = Twitter(domain='upload.twitter.com',
	    auth=OAuth(ACCESS_TOKEN, ACCESS_SECRET, CONSUMER_KEY, CONSUMER_SECRET))

	if take_three:
		id_img1 = t_up.media.upload(media=imagedata1)["media_id_string"]
		id_img2 = t_up.media.upload(media=imagedata2)["media_id_string"]
		id_img3 = t_up.media.upload(media=imagedata3)["media_id_string"]
		t.statuses.update(status=random.choice(phrases), media_ids=",".join([id_img1, id_img2, id_img3]))
	else:
		params = {"media[]": imagedata1, "status": random.choice(phrases)}
		t.statuses.update_with_media(**params)

