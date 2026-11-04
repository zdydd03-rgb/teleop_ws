from pathlib import Path
from setuptools import find_packages, setup

package='pico_o7'
data=[('share/ament_index/resource_index/packages',['resource/'+package]),
      ('share/'+package,['package.xml','README.md','requirements.lock','sources_manifest.json','VENDOR_NOTES.md'])]
for folder in ('launch','config','models','calibration','vendor'):
    groups={}
    for path in Path(folder).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.suffix!='.pyc':
            groups.setdefault(str(Path('share')/package/path.parent),[]).append(str(path))
    data.extend(sorted(groups.items()))
setup(name=package,version='1.0.0',packages=find_packages(exclude=['test']),
      data_files=data,install_requires=['setuptools'],zip_safe=False,
      maintainer='RM Teleop Developer',maintainer_email='user@example.com',
      description='PICO O7 retargeting on the existing RM65 session',license='Apache-2.0',
      tests_require=['pytest'],entry_points={'console_scripts':[
          'pico_hand_receiver_node = pico_o7.pico_hand_receiver_node:main',
          'shared_session_node = pico_o7.shared_session_node:main',
          'o7_session_tools = pico_o7.session_tools:main',
      ]})
