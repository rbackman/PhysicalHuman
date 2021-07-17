# GSim Skeleton Definition - M. Kallmann 2006

KnSkeleton

name primohd_hand

geofile "primohd_hand.m"

skeleton
root RightHand
{ offset 0 0 0
  visgeo "primohd_hand_righthand_vis.m"
  channel XPos 0 free
  channel YPos 0 free
  channel ZPos 0 free
  euler XYZ
  channel XRot 0 free
  channel YRot 0 free
  channel ZRot 0 free

  joint RightHandIndex1
  { offset -3.5995 0.1681 0.81096
    visgeo "primohd_hand_righthandindex1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandIndex2
    { offset -1.759 0 0
      visgeo "primohd_hand_righthandindex2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandIndex3
      { offset -1.0011 0 0
        visgeo "primohd_hand_righthandindex3_vis.m"
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
  { offset -3.4777 0.1944 -0.03772
    visgeo "primohd_hand_righthandmiddle1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandMiddle2
    { offset -1.8807 0 0
      visgeo "primohd_hand_righthandmiddle2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandMiddle3
      { offset -1.1105 0 0
        visgeo "primohd_hand_righthandmiddle3_vis.m"
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
  { offset -3.1834 -0.3504 -1.61046
    visgeo "primohd_hand_righthandpinky1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandPinky2
    { offset -1.1727 0 0
      visgeo "primohd_hand_righthandpinky2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandPinky3
      { offset -0.8552 0 0
        visgeo "primohd_hand_righthandpinky3_vis.m"
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
  { offset -3.398 -0.0271 -0.86476
    visgeo "primohd_hand_righthandring1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandRing2
    { offset -1.8628 0 0
      visgeo "primohd_hand_righthandring2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandRing3
      { offset -0.9836 0 0
        visgeo "primohd_hand_righthandring3_vis.m"
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
  { offset -0.9834 -0.3812 1.25818
    visgeo "primohd_hand_righthandthumb1_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightHandThumb2
    { offset 0 0.0086 1.66202
      visgeo "primohd_hand_righthandthumb2_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightHandThumb3
      { offset 0 0.0055 1.06878
        visgeo "primohd_hand_righthandthumb3_vis.m"
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
