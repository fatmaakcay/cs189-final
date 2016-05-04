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

def post_twitter(file_name):


	t = Twitter(
	    auth=OAuth(ACCESS_TOKEN, ACCESS_SECRET, CONSUMER_KEY, CONSUMER_SECRET))



	with open(file_name, "rb") as imagefile:
	    imagedata = imagefile.read()

	t_up = Twitter(domain='upload.twitter.com',
	    auth=OAuth(ACCESS_TOKEN, ACCESS_SECRET, CONSUMER_KEY, CONSUMER_SECRET))
	# id_img1 = t_up.media.upload(media=imagedata)["media_id_string"]
	# id_img2 = t_up.media.upload(media=imagedata)["media_id_string"]
	# # - finally send your tweet with the list of media ids:
	# t.statuses.update(status="PTT *", media_ids=",".join([id_img1, id_img2]))

	# Or send a tweet with an image (or set a logo/banner similarily)
	# using the old deprecated method that will probably disappear some day
	params = {"media[]": imagedata, "status": random.choice(phrases)}

	t.statuses.update_with_media(**params)


# Get your "home" timeline
# t.statuses.home_timeline()

# Get a particular friend's timeline
# t.statuses.user_timeline(screen_name="billybob")

# to pass in GET/POST parameters, such as `count`
# t.statuses.home_timeline(count=5)

# to pass in the GET/POST parameter `id` you need to use `_id`
# t.statuses.oembed(_id=1234567890)

# Update your status
# t.statuses.update(
#     status="Hello World!")

# Send a direct message
# t.direct_messages.new(
#     user="billybob",
#     text="I think yer swell!")

# Get the members of tamtar's list "Things That Are Rad"
# t.lists.members(owner_screen_name="tamtar", slug="things-that-are-rad")

# An *optional* `_timeout` parameter can also be used for API
# calls which take much more time than normal or twitter stops
# # responding for some reason:
# t.users.lookup(
#     screen_name=','.join(A_LIST_OF_100_SCREEN_NAMES), _timeout=1)

# Overriding Method: GET/POST
# you should not need to use this method as this library properly
# detects whether GET or POST should be used, Nevertheless
# to force a particular method, use `_method`
# t.statuses.oembed(_id=1234567890, _method='GET')



# ACCESS_TOKEN = '2891467479-eGI4aodebSYX3wV6va7se1hUgkp50e5Os9Uc3y1'
# ACCESS_SECRET = 'tRyAlxBJr8cRRn6fASGPJq5oHtoSUdKUPlxX4U5rIiXsL'
# CONSUMER_KEY = 'EUj3QXXrbwSo9TnxjseOrVTXc'
# CONSUMER_SECRET = 'qdUorN97isnCGW2uVNeX6eTwAtcVvntABwmM8IOpxcAE3etLGz'