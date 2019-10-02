# How to install Python 3 and OpenCV 4 for Windows 10

I was able to install Python 3 and OpenCV 4 under Windows 10
by following the steps below.

I. Install Python 3.7.4:
------------------------

1. Go to:

   https://www.python.org/downloads/windows

2. Click on link "Windows x86-64 web-based installer"
   Save file somewhere (Downloads)

3. Run: python-3.7.4-amd64-webinstall.exe

4. Click on "Update PATH"

5. Click on "Install Now"

6. When it is finished, click close.

It is very important that you choose the option to update your PATH. If you forget to,
just re-install.

II. Install Additional Packages
-------------------------------

Open Power Shell or DOS Command Prompt and use
pip to install additional packages:

``` shell
   python -m pip install --upgrade pip
   pip install numpy
   pip install scipy
   pip install matplotlib
   pip install scikit-learn
   pip install scikit-image
```


III. Download OPenCV
--------------------

Go to:

      https://www.lfd.uci.edu/~gohlke/pythonlibs/#opencv

Find the opencv package for OpenCV 4.1.1 and python 3.7 (64-bit version):

     opencv_python‑4.1.1+contrib‑cp37‑cp37m‑win_amd64.whl

and click to download OpenCV.

I would give a direct link, but the link just runs some javascript to do the download.


IV. Install OpenCV
------------------

1. Open powershell or DOS Command Prompt.
2. Navigate to the directory where you downloaded
   the OpenCV "whl" file above.
3. Run:

``` shell
        pip install .\opencv_python‑4.1.1+contrib‑cp37‑cp37m‑win_amd64.whl
```

V. Verify the installation.

1. Open PowerShell or DOS Command Prompt and run:

``` shell
     python
```

You should see a message about "Python 3.7.4" and a prompt ">>>".
At the prompt, type:

``` shell
   import cv2
   import numpy
   exit()
```

If you don't get an error when you input "import cv2", cv2 is installed.


Trouble
-------

The only trouble that I encountered was that I had to turn on access
in: Settings | Privacy | Camera. I couldn't access the camera until
after I turned access on and allowed desktop apps to access the camera.

When I was having trouble, I tried using the generic "USB Video Device" instead of the "HD Pro Webcam C920"
but that didn't have any effect. After I got the camera working, I switched back to the "HD Pro Webcam C920"
driver and Python/OpenCV continued to work.
