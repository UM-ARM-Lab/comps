#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/elastic_strips')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    ElasticStrips = RaveCreateModule(env,'ElasticStrips')
    print ElasticStrips.SendCommand('help')
finally:
    RaveDestroy()
