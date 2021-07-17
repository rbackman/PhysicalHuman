# GSim Skeleton Definition - M. Kallmann 2006

KnSkeleton

name Mentar

skeleton
root Hips
{ offset 0 0 0
  channel XPos 0 free
  channel YPos 0 free
  channel ZPos 0 free
  channel Quat

  joint LeftUpLeg
  { offset 0.139755 0 0
    channel Quat

    joint LeftLeg
    { offset 0 -0.559019 0
      channel Quat

      joint LeftFoot
      { offset 0 -0.512434 0
        channel Quat

        joint LeftToeBase
        { offset 0 -0.0527497 0.196864
          channel Quat

          joint EndSite
          { offset 0 0 0.0873467
          }
        }
      }
    }
  }

  joint RightUpLeg
  { offset -0.139755 0 0
    channel Quat

    joint RightLeg
    { offset 0 -0.559019 0
      channel Quat

      joint RightFoot
      { offset 0 -0.512434 0
        channel Quat

        joint RightToeBase
        { offset 0 -0.0527497 0.196864
          channel Quat

          joint EndSite
          { offset 0 0 0.0873467
          }
        }
      }
    }
  }

  joint Spine
  { offset 0 0.00116462 0
    channel Quat

    joint Spine1
    { offset 0 0.273686 0
      channel Quat

      joint Neck
      { offset 0 0.256217 0
        channel Quat

        joint Head
        { offset 0 0.133932 0
          channel Quat

          joint EndSite
          { offset 0 0.133932 0
          }
        }
      }

      joint LeftShoulder
      { offset 0 0.256217 0
        channel Quat

        joint LeftArm
        { offset 0.174693 0 0
          channel Quat

          joint LeftForeArm
          { offset 0.349387 0 0
            channel Quat

            joint LeftHand
            { offset 0.320271 0 0
              channel Quat

              joint LeftHandThumb
              { offset 0 0 0
                channel Quat

                joint EndSite
                { offset 0 0 0.116462
                }
              }

              joint L_Wrist_End
              { offset 0.116462 0 0
                channel Quat
              }
            }
          }
        }
      }

      joint RightShoulder
      { offset 0 0.256217 0
        channel Quat

        joint RightArm
        { offset -0.174693 0 0
          channel Quat

          joint RightForeArm
          { offset -0.349387 0 0
            channel Quat

            joint RightHand
            { offset -0.320271 0 0
              channel Quat

              joint RightHandThumb
              { offset 0 0 0
                channel Quat

                joint EndSite
                { offset 0 0 0.116462
                }
              }

              joint R_Wrist_End
              { offset -0.160136 0 0
                channel Quat
              }
            }
          }
        }
      }
    }
  }
}

end
