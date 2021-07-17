# GSim Skeleton Definition - M. Kallmann 2006

KnSkeleton

name RightHand

geofile "RightHand.m"

skeleton
root RightHand
{ offset 0 0 0
  visgeo "righthand_righthand_vis.m"
  channel XPos 0 free
  channel YPos 0 free
  channel ZPos 0 free
  euler XYZ
  channel XRot 0 free
  channel YRot 0 free
  channel ZRot 0 free

  joint RightHandIndex1
  { offset -3.23211 0.12562 1.0209
    visgeo "righthand_righthandindex1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandIndex2
    { offset -1.759 0 0
      visgeo "righthand_righthandindex2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandIndex3
      { offset -1.0011 0 0
        visgeo "righthand_righthandindex3_vis.m"
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint RightHandIndex4
        { offset -1.2593 0 0
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free
        }
      }
    }
  }

  joint RightHandMiddle1
  { offset -3.32025 0.15192 0.03226
    visgeo "righthand_righthandmiddle1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandMiddle2
    { offset -1.8807 0 0
      visgeo "righthand_righthandmiddle2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandMiddle3
      { offset -1.1105 0 0
        visgeo "righthand_righthandmiddle3_vis.m"
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint RightHandMiddle4
        { offset -1.2862 0 0
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free
        }
      }
    }
  }

  joint RightHandPinky1
  { offset -3.27087 -0.39288 -1.66294
    visgeo "righthand_righthandpinky1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandPinky2
    { offset -1.1727 0 0
      visgeo "righthand_righthandpinky2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandPinky3
      { offset -0.8552 0 0
        visgeo "righthand_righthandpinky3_vis.m"
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint RightHandPinky4
        { offset -1.118 0 0
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free
        }
      }
    }
  }

  joint RightHandRing1
  { offset -3.32802 -0.06958 -0.86476
    visgeo "righthand_righthandring1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandRing2
    { offset -1.8628 0 0
      visgeo "righthand_righthandring2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandRing3
      { offset -0.9836 0 0
        visgeo "righthand_righthandring3_vis.m"
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint RightHandRing4
        { offset -1.0897 0 0
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free
        }
      }
    }
  }

  joint RightHandThumb1
  { offset -1.45575 -0.3812 1.45062
    visgeo "righthand_righthandthumb1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandThumb2
    { offset 0 0.0086 1.66202
      visgeo "righthand_righthandthumb2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandThumb3
      { offset 0 0.0055 1.06878
        visgeo "righthand_righthandthumb3_vis.m"
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint RightHandThumb4
        { offset 0 0.0062 1.2068
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free
        }
      }
    }
  }
}

end
