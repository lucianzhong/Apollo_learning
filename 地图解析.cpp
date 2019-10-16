
// https://blog.csdn.net/lewif/article/details/78575840


modules/map/hdmap/adapter/odr_adapter.cc



      // For right-side lanes, the lane link defination in hdmap same as Opendrive defines,
      // only need to consider is Opendrive's Current Road's direction is same as the Next Road or not.
      // There are two cases:
      // CASE 1: -->-->--> Cur Road -->-->--> (LINK) -->-->--> Next Road -->-->-->
      // CASE 2: -->-->--> Cur Road -->-->--> (LINK) <--<--<-- Next Road <--<--<--
      // For Case 1, need to link the Current Road's end section to the Next road's start section,
      // For Case 2, need to link the Current Road's end section to the Next road's end section.




      // For left-side lanes, the lane link defination in hdmap opposite to Opendrive defines,



Lanes that can be used by traffic as drivable or parking lanes of one-way roads must have only IDs corresponding to the intended driving direction:
◦ negative IDs for right hand traffic in track orientation and left hand traffic opposite to track orientation
◦ positive IDs for right hand traffic opposite to track
orientation and left hand traffic in track orientation 

// http://www.opendrive.org/docs/OpenDRIVEStyleGuideRevC.pdf



Lanes are numbered in ascending order according to their distance from the chord line. The total
number of lanes is not limited. The chord line itself is defined as lane zero and must not have a width
entry (i.e. the width must always be 0.0).
Lanes to the left of the chord line are positive, lanes to the right are negative. A road layout with two
lanes per direction, median and shoulders is shown in the following figure (the road is running from left
to right, see s co-ordinate):Lanes are numbered in ascending order according to their distance from the chord line. The total
number of lanes is not limited. The chord line itself is defined as lane zero and must not have a width
entry (i.e. the width must always be 0.0).
Lanes to the left of the chord line are positive, lanes to the right are negative. A road layout with two
lanes per direction, median and shoulders is shown in the following figure (the road is running from left
to right, see s co-ordinate):

// http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.1D.pdf


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
The driving direction:

(1) The left lanes of the reference line based alongside the driving direction should be postive. 
(2) For the right hand dring priority, the negative ID lanes should be driving in the same dirction as the reference line.

