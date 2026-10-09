# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
# What do these roots pull from the SDK archives, and what stays undefined?
#   closure.py LIBDIR a.a,b.a root1,root2 OUT.txt      (needs llvm-nm that reads the SDK bitcode)
import subprocess, sys, re, collections, os
TC="/usr/bin/llvm-nm"
LIB=sys.argv[1]; archives=sys.argv[2].split(","); roots=sys.argv[3].split(",")
defs={}      # symbol -> (archive, member)
members={}   # (archive, member) -> (defined set, undefined set)
for a in archives:
    out=subprocess.run([TC,"-A","-g","--no-demangle",f"{LIB}/{a}"],capture_output=True,text=True).stdout
    for l in out.splitlines():
        m=re.match(r'^.*?([^/]+\.a):(\S+?):\s+[0-9a-f-]*\s*(\w) (\S+)$',l)
        if not m: continue
        arc,mem,t,sym=m.groups(); k=(arc,mem)
        d,u=members.setdefault(k,(set(),set()))
        if t in "Uvw": u.add(sym)
        elif t!="U": d.add(sym); defs.setdefault(sym,k)
        if t=="U": pass
need=list(roots); pulled=set(); missing=collections.Counter()
while need:
    s=need.pop()
    k=defs.get(s)
    if not k: missing[s]+=1; continue
    if k in pulled: continue
    pulled.add(k)
    need.extend(members[k][1])
sizes=collections.Counter()
for (arc,mem) in pulled: sizes[arc]+=1
print("members pulled:",dict(sizes),"total",len(pulled))
print("undefined (not in given archives):",len(missing))
open(sys.argv[4],"w").write("\n".join(sorted(missing))+"\n")
