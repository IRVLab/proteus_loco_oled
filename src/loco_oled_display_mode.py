#! /usr/bin/python3

import sys, math, threading, signal
from time import sleep
from math import pi

from loco_oled.ssd1325 import SSD1325

import rospy
from rosnode import get_node_names
from rospy import service
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import xml.etree.ElementTree as ET
from proteus.vector.oled import DisplayPhrase, DNode
from proteus_msgs.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity

# rospy.init_node('loco_oled_server', argv=None, anonymous=True)

oled = SSD1325()
oled.configure_dimmensions()
oled.clear_display()
oled.draw_loco_logo()
oled.display()

oled.set_text_size(2)
oled.set_text_color() # Important to remmeber to actually set text color.

def service_cb(req, display_phrase):
    rospy.logdebug('Service callback for display phrase %s'%(display_phrase.id))
    if display_phrase.call_type == 'trigger':
        return execute_trigger(req, display_phrase)
    elif display_phrase.call_type == 'directional':
        return execute_directional(req, display_phrase)
    elif display_phrase.call_type == 'target':
        return execute_target(req, display_phrase)
    elif display_phrase.call_type == 'quantity':
        return execute_quantity(req, display_phrase)
    else:
        return False

def execute_trigger(req, display_phrase):
    for d in display_phrase.dnodes:
        draw_on_oled(d.text.data)

        # Wait for the appropriate time
        sleep(d.duration.seconds) 

        # Clear the display
        oled.clear_display() #Clean up the buffer (the reason we do this and pre-clear is that we can't trust other code to clear the buffer for us.)
        # oled.display()

    return True

def execute_directional(req, display_phrase):
    direction = cardinalize(req.transform)
    for d in display_phrase.dnodes:
        draw_on_oled(d.text.data.format(direction))

        # Wait for the appropriate time
        sleep(d.duration.seconds) 

        # Clear the display
        oled.clear_display() #Clean up the buffer (the reason we do this and pre-clear is that we can't trust other code to clear the buffer for us.)
        # oled.display()

    return True

def execute_target(req, display_phrase):
    return False

def execute_quantity(req, display_phrase):
    quant = req.quantity * 100
    for d in display_phrase.dnodes:
        draw_on_oled(d.text.data.format(quant))

        # Wait for the appropriate time
        sleep(d.duration.seconds) 

        # Clear the display
        oled.clear_display() #Clean up the buffer (the reason we do this and pre-clear is that we can't trust other code to clear the buffer for us.)
        # oled.display()

    return True

def cardinalize(transform):
    q = transform.rotation
    rpy = euler_from_quaternion([q.x, q.y, q.z, q.w]) #We only actually need pitch and yaw, roll is ignored here.

    ret = "" # Return string.

    # Pitch handling
    if rpy[1] > 0:
        ret+= "up"
    elif rpy[1] < 0:
        ret+= "down"

    # Add a conjunction if there's pitch involved.
    if rpy[1] != 0 and rpy[2] != 0:
        ret+= " and "

    # Yaw handling
    if rpy[2] > 0:
        ret+= "left"
    elif rpy[2] < 0:
        ret += "right"

    return ret

def draw_on_oled(text):
    line_size = 9
    lines = list()
    l = ""

    words = text.split(" ")
    word_ls = [len(w) for w in words]

    if (len(words) < 4) and (max(word_ls) < 11):
        # If every word will fit on its own line, just make it happen
        lines.extend(words)
    elif (len(words) == 1) and (word_ls[0] > 8):
        midpoint = int(word_ls[0]/3) * 2
        word = words[0]
        lines.append(word[0:midpoint])
        lines.append(word[midpoint:])

    else:
        fill = 0
        for k, w in enumerate(words):
            word_len = word_ls[k]
            if (word_len + 1) < (line_size - fill):
                # Enough space for word and space
                l += w + " "
                fill += (word_len + 1)
            elif word_len < (line_size - fill):
                # Enough space for word only, so put in in, then start a new line
                l += w
                lines.append(l)

                l = ""
                fill = 0
            else:
                #Not enough space in the line for the whole word
                space = line_size - fill
                if space < 3:
                    # Only 1-2 letters could fit on this line, put the whole word on the next line
                    lines.append(l)

                    l = w + " "
                    fill = word_ls[k] + 1
                else:
                    # More than 2 letters could fit on this line, split the word and go to next line.
                    l += w[0:space+1]

                    lines.append(l)
                    l = w[space+1:]
                    fill = len(l)


        lines.append(l) # Get the last line in there.

    row = 5
    for l in lines:
        # Pad either side of the line with spaces.
        flip = True
        while len(l) < line_size:
            if flip:
                l = " " + l
                flip = False
            else:
                l += " "
                flip = True

        oled.set_cursor(0, row)
        oled.print_line(l)

        row += 20

    return

if __name__ == '__main__':
    rospy.loginfo('Initializing the LoCO OLED server')

    #Check if PROTEUS language server is up
    rospy.loginfo('Checking PROTEUS language server...')
    lang_server_active = False
    nodes = get_node_names()
    rospy.logdebug(nodes)
    for n in nodes:
        if n.split('/')[-1] == 'proteus_language_server':
            lang_server_active = True
            break
    if not lang_server_active:
        rospy.logerr("This OLED display implementation requires the PROTEUS language server to be active.")
        sys.exit(1)
    else:
        rospy.loginfo('PROTEUS language server OK!')

    # Find kineme language definition file
    rospy.loginfo("Loading digital display vector information...")
    dd_info = rospy.get_param('vectors/out/DigitalDisplay')
    dd_def_file = dd_info['definition_file']

    # Find symbol definitions
    rospy.loginfo("Loading symbol information...")
    symbols = rospy.get_param('symbols/out')
    
    # Process display definition file into kineme objects
    rospy.loginfo("Loading display definitions from display definition file.")
    disp_phrases = dict()

    #Load XML file
    tree = ET.parse(dd_def_file)
    root = tree.getroot()
    for dpdef in root:
        d = DisplayPhrase()
        d.parse_from_xml(dpdef)
        disp_phrases[d.id] = d

    # Check for symbol matchup.
    for s in symbols:
        for key in disp_phrases:
            d = disp_phrases[key]
            if s == d.id:
                rospy.loginfo("Found match beteween symbol %s and display phrase %s, associating data."%(s, d.id))
                rospy.logdebug("Call type: %s"%(symbols.get(s).get('call_type')))
                d.set_call_type(symbols.get(s).get('call_type'))
                break

    print(disp_phrases)

    # Setup service calls
    for key, disp in disp_phrases.items():
        service_class = None
        if disp.call_type == 'trigger':
            service_class = SymbolTrigger
        elif disp.call_type == 'directional':
            service_class = SymbolDirectional
        elif disp.call_type == 'target':
            service_class = SymbolTarget
        elif disp.call_type == 'quantity':
            service_class = SymbolQuantity
        else:
            rospy.logwarn("Unexpected call type {} for display-phrase {}".format(disp.call_type, disp.name))

        service_name = 'digital_display/'+ disp.name.replace(' ', '_')

        rospy.loginfo('Advertising a service for display phrase %s at service endpoint: %s'%(disp.id, service_name))
        rospy.Service(service_name, service_class, lambda req, disp=disp: service_cb(req, disp))

    sleep(5)
    oled.clear_display()
    oled.display()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        oled.display() # Draw whatever is in the buffer.
        rate.sleep()

else:
    pass
