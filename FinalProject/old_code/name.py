import sys, select

if True:
    print "Press Enter if you'd like to take a picture!"

    i, o, e = select.select( [sys.stdin], [], [], 10)

    if (i):

        wantsPhoto = True
        print "yay"
        # print "You said " + sys.stdin.readline().strip()


    else:
        wantsPhoto = False 
        print "Okay, maybe I'll find you next time!"