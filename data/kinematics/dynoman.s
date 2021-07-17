# GSim Skeleton Definition - M. Kallmann 2006

KnSkeleton

name dynoman

geofile "dynoman.m"

skeleton
root Hips
{ offset 0 0 0
  channel XPos 0 free
  channel YPos 0 free
  channel ZPos 0 free
  euler XYZ
  channel XRot 0 free
  channel YRot 0 free
  channel ZRot 0 free

  joint Spine
  { offset 0 0.02813 0
    visgeo "dynoman_spine_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint Spine1
    { offset 0 0.27369 0
      visgeo "dynoman_spine1_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint Neck
      { offset 0 0.25622 0
        visgeo "dynoman_neck_vis.m"
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint Head
        { offset 0 0.13393 0
          visgeo "dynoman_head_vis.m"
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free
        }
      }

      joint LeftShoulder
      { offset 0 0.25622 0
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint LeftArm
        { offset 0.17469 0 0
          visgeo "dynoman_leftarm_vis.m"
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free

          joint LeftArmTwist
          { offset 0 0 0
            visgeo "dynoman_leftarmtwist_vis.m"
            channel XRot 0 free
            channel YRot 0 free
            channel ZRot 0 free

            joint LeftForeArm
            { offset 0.34939 0 0
              visgeo "dynoman_leftforearm_vis.m"
              channel XRot 0 free
              channel YRot 0 free
              channel ZRot 0 free

              joint LeftHand
              { offset 0.32027 0 0
                visgeo "dynoman_lefthand_vis.m"
                channel XRot 0 free
                channel YRot 0 free
                channel ZRot 0 free

                joint LeftHandTwist
                { offset -0.00143 0 0
                  visgeo "dynoman_lefthandtwist_vis.m"
                  channel XRot 0 free
                  channel YRot 0 free
                  channel ZRot 0 free

                  joint LeftFinger
                  { offset 0.09883 0 0
                    visgeo "dynoman_leftfinger_vis.m"
                    channel XRot 0 free
                    channel YRot 0 free
                    channel ZRot 0 free

                    joint LeftFingerEnd
                    { offset 0.06568 0 0
                      channel XRot 0 free
                      channel YRot 0 free
                      channel ZRot 0 free
                    }
                  }

                  joint LeftThumb
                  { offset 0.00143 0 0.05791
                    visgeo "dynoman_leftthumb_vis.m"
                    channel XRot 0 free
                    channel YRot 0 free
                    channel ZRot 0 free

                    joint LeftThumbTwist
                    { offset 0 0 0
                      visgeo "dynoman_leftthumbtwist_vis.m"
                      channel XRot 0 free
                      channel YRot 0 free
                      channel ZRot 0 free

                      joint LeftThumbEnd
                      { offset 0 0 0.085
                        channel XRot 0 free
                        channel YRot 0 free
                        channel ZRot 0 free
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }

      joint RightShoulder
      { offset 0 0.25622 0
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint RightArm
        { offset -0.17469 0 0
          visgeo "dynoman_rightarm_vis.m"
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free

          joint RightArmTwist
          { offset 0 0 0
            visgeo "dynoman_rightarmtwist_vis.m"
            channel XRot 0 free
            channel YRot 0 free
            channel ZRot 0 free

            joint RightForeArm
            { offset -0.34939 0 0
              visgeo "dynoman_rightforearm_vis.m"
              channel XRot 0 free
              channel YRot 0 free
              channel ZRot 0 free

              joint RightHand
              { offset -0.32027 0 0
                visgeo "dynoman_righthand_vis.m"
                channel XRot 0 free
                channel YRot 0 free
                channel ZRot 0 free

                joint RightHandTwist
                { offset 0.00143 0 0
                  visgeo "dynoman_righthandtwist_vis.m"
                  channel XRot 0 free
                  channel YRot 0 free
                  channel ZRot 0 free

                  joint RightFinger
                  { offset -0.09883 0 0
                    visgeo "dynoman_rightfinger_vis.m"
                    channel XRot 0 free
                    channel YRot 0 free
                    channel ZRot 0 free

                    joint RightFingerEnd
                    { offset -0.06568 0 0
                      channel XRot 0 free
                      channel YRot 0 free
                      channel ZRot 0 free
                    }
                  }

                  joint RightThumb
                  { offset -0.00143 0 0.05791
                    visgeo "dynoman_rightthumb_vis.m"
                    channel XRot 0 free
                    channel YRot 0 free
                    channel ZRot 0 free

                    joint RightThumbTwist
                    { offset 0 0 0
                      visgeo "dynoman_rightthumbtwist_vis.m"
                      channel XRot 0 free
                      channel YRot 0 free
                      channel ZRot 0 free

                      joint RightThumbEnd
                      { offset 0 0 0.085
                        channel XRot 0 free
                        channel YRot 0 free
                        channel ZRot 0 free
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  joint LeftLegwist
  { offset 0.13975 0.02697 0
    visgeo "dynoman_leftlegwist_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint LeftUpLeg
    { offset 0 0 0
      visgeo "dynoman_leftupleg_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint LeftLeg
      { offset 0 -0.55902 0
        visgeo "dynoman_leftleg_vis.m"
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint LeftFoot
        { offset 0 -0.51243 0
          visgeo "dynoman_leftfoot_vis.m"
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free

          joint LeftToeBase
          { offset 0 -0.05275 0.19686
            visgeo "dynoman_lefttoebase_vis.m"
            channel XRot 0 free
            channel YRot 0 free
            channel ZRot 0 free

            joint LeftToeEnd
            { offset 0 0 0.08735
              channel XRot 0 free
              channel YRot 0 free
              channel ZRot 0 free
            }
          }
        }
      }
    }
  }

  joint RightLegTwist
  { offset -0.13976 0.02697 0
    visgeo "dynoman_rightlegtwist_vis.m"
    channel XRot 0 free
    channel YRot 0 free
    channel ZRot 0 free

    joint RightUpLeg
    { offset 0 0 0
      visgeo "dynoman_rightupleg_vis.m"
      channel XRot 0 free
      channel YRot 0 free
      channel ZRot 0 free

      joint RightLeg
      { offset 0 -0.55902 0
        visgeo "dynoman_rightleg_vis.m"
        channel XRot 0 free
        channel YRot 0 free
        channel ZRot 0 free

        joint RightFoot
        { offset 0 -0.51243 0
          visgeo "dynoman_rightfoot_vis.m"
          channel XRot 0 free
          channel YRot 0 free
          channel ZRot 0 free

          joint RightToeBase
          { offset 0 -0.05275 0.19686
            visgeo "dynoman_righttoebase_vis.m"
            channel XRot 0 free
            channel YRot 0 free
            channel ZRot 0 free

            joint RightToeEnd
            { offset 0 0 0.08735
              channel XRot 0 free
              channel YRot 0 free
              channel ZRot 0 free
            }
          }
        }
      }
    }
  }
}

end
