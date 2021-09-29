Import("env")
import os
print ("Pre build script")
os.system('sh -c "./getlibs.sh"')


#def pre_build(source, target, env):

#env.AddPreAction("getlibs", pre_build)
