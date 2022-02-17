#! /usr/bin/python3 

import sys, math, threading, signal
from time import sleep
from math import pi

from loco_oled.ssd1325 import SSD1325

import rospy
from rosnode import get_node_names
from rospy import service
from geometry_msgs.msg import Transform

import xml.etree.ElementTree as ET
from proteus.display_phrase import DisplayPhrase, DNode
from proteus.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity

# rospy.init_node('loco_oled_server', argv=None, anonymous=True)

oled = SSD1325()
oled.configure_dimmensions()
oled.clear_display()
oled.draw_loco_logo()
oled.display()

oled.set_text_size(2)
oled.set_text_color()

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
        oled.set_cursor(0,20)
        oled.clear_display() #flush any previous data away
        oled.print_line(d.text.data) # print a line
        rospy.loginfo(d.text.data)
        oled.display() #This function draws what's in the buffer.

        # Wait for the appropriate time
        sleep(d.duration.seconds) 

        # Clear the display
        oled.clear_display() #Clean up the buffer (the reason we do this and pre-clear is that we can't trust other code to clear the buffer for us.)
        oled.display()

    return True

def execute_directional(req, display_phrase):
    direction = cardinalize(req.transform)
    for d in display_phrase.dnodes:
        oled.set_cursor(0,0)
        oled.clear_display() #flush any previous data away
        oled.print_line(d.text.data.format(direction)) # print a line
        # oled.display() #This function draws what's in the buffer.

        # Wait for the appropriate time
        sleep(d.duration.seconds) 

        # Clear the display
        oled.clear_display() #Clean up the buffer (the reason we do this and pre-clear is that we can't trust other code to clear the buffer for us.)
        # oled.display()

    return True

def execute_target(req, display_phrase):
    return False

def execute_quantity(req, display_phrase):
    quant = req.quantity
    for d in display_phrase.dnodes:
        oled.set_cursor(0,0)
        oled.clear_display() #flush any previous data away
        oled.print_line(d.text.data.format(quant)) # print a line
        # oled.display() #This function draws what's in the buffer.

        # Wait for the appropriate time
        sleep(d.duration.seconds) 

        # Clear the display
        oled.clear_display() #Clean up the buffer (the reason we do this and pre-clear is that we can't trust other code to clear the buffer for us.)
        # oled.display()

    return True

def cardinalize(transform):
    return "left"

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
        # oled.display() # Draw whatever is in the buffer.
        rate.sleep()

else:
    pass