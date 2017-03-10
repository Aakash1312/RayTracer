(Material sphere1mat
    (color .4 .5 .9)
    (ma .1) (ml .5) (ms 1000.5) (msp 5) (msm 1) (mt 0) (mtn 1)
)

(Material sphere2mat
    (color .9 .4 .5)
    (ma .4) (ml .2) (ms 1500.5) (msp 5) (msm 1) (mt 0) (mtn 1)
)

(Material sphere3mat
    (color .5 .9 .4)
    (ma .5) (ml .5) (ms 2000.3) (msp 5) (msm 1) (mt 0) (mtn 1)
)

(Sphere sph1 (radius 1) (material sphere1mat))
(Sphere sph2 (radius 2.5) (material sphere2mat))
(Sphere sph3 (radius 1.0) (material sphere3mat))

(Light amb
    (type 0) (color .5 .2 .2)
)

(Light dir
    (type 1) (color .4 .8 1.2)
)

(Light pt
    (type 2) (color 1.39 .2 .2) (falloff 0.1) (deaddistance 3)
)

(Camera cam
    (perspective 1)
    (l -1) (r 1) (b -1) (t 1) (n 3) # no far plane 
)

(G threespheres
    (I cam1 cam)
    (I dir1 dir (Xform 0 0 -.5 0
                           0 0 -.5 0
                           0 0 .5 0
                           0 0 0 0)) # points toward (.5,.5,-.5)
    (I amb1 amb)
    (I pt1 pt (T 0 0 -14))
    (I s1 sph1 (T -2.5 -1.5 -17.0))
    (I s2 sph2 (T 0 4.0 -25))
    (I s3 sph3 (T 1.5 -1.5 -10.0))
)

(Render  threespheres)


