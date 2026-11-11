#!/usr/bin/env python3
"""Read-only V1 HIL verification: SQLite CDR String payloads and CSV; no ROS/SDK imports."""
from pathlib import Path
import csv,datetime,hashlib,json,sqlite3,statistics,struct
from collections import Counter
R=Path(__file__).resolve().parents[3]
B=R/'concurrency_logs/o7_combined_10hz_20261111_172614'
C=R/'canfd_logs/rm65_canfd_high_follow_20261111_172612.csv'
L=Path(__file__).parent/'raw_logs/python3_29026_1794389172660.log'
def summary(values):
 v=sorted(values)
 def q(p):
  x=(len(v)-1)*p;i=int(x);return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(x-i)
 return dict(n=len(v),mean=statistics.mean(v),p50=q(.5),p95=q(.95),p99=q(.99),max=max(v)) if v else None
con=sqlite3.connect((B/next(B.glob('*.db3')).name).as_uri()+'?mode=ro',uri=True)
a,b=con.execute('select min(timestamp),max(timestamp) from messages').fetchone()
tid=con.execute("select id from topics where name='/rm65/o7_modbus_hil/event'").fetchone()[0]
events=[]
for data, in con.execute('select data from messages where topic_id=? order by timestamp',(tid,)):
 # CDR encapsulation followed by the single String member (length includes NUL).
 endian='<' if data[1]&1 else '>';n=struct.unpack_from(endian+'I',data,4)[0]
 events.append(json.loads(data[8:8+n-1].decode()))
w=[e for e in events if e['event']=='O7_HAND_WRITE_FINISH']
rows=[r for r in csv.DictReader(C.open()) if a<=int(datetime.datetime.fromisoformat(r['local_time']).timestamp()*1e9)<=b]
sent=[r for r in rows if r['state']=='ACTIVE' and r['command_sent']=='True']
ints=[float(r['command_interval_sec'])*1000 for r in sent if float(r['command_interval_sec'])>=0]
init=[]
for line in L.read_text().splitlines():
 if 'O7 HIL {' in line:
  e=json.loads(line.split('O7 HIL ',1)[1])
  if e['event'] in ('O7_MODBUS_INIT_SUCCESS','O7_MODBUS_INIT_FAILED'):init.append(e)
files=[*B.iterdir(),C,*L.parent.glob('*.log')]
print(json.dumps(dict(bag_start_ns=a,bag_end_ns=b,o7_calls=len(w),o7_codes=dict(Counter(str(e['return_code']) for e in w)),
 o7_duration_ms=summary([e['duration_ms'] for e in w]),o7_hz=(len(w)-1)/((w[-1]['call_start_timestamp_ns']-w[0]['call_start_timestamp_ns'])/1e9),
 canfd_sent=len(sent),canfd_interval_ms=summary(ints),over_ms={str(k):sum(x>k for x in ints) for k in (5,10,20)},
 canfd_codes=dict(Counter(r['movej_canfd_return_code'] for r in sent)),ik_codes=dict(Counter(r['ik_return_code'] for r in sent)),
 fault_rows=sum(r['fault_latched']=='True' for r in rows),stop_rows=sum(r['stop_command_sent']=='True' for r in rows),initializations=init,
 files=[dict(path=str(p.relative_to(R)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files if p.is_file()]),ensure_ascii=False,indent=2))
