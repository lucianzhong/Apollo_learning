 
 
 1. // 前言
 
高精度电子地图也称为高分辨率地图(HD Map，High Definition Map)，是一种专门为无人驾驶服务的地图。与传统导航地图不同的是，高精度地图除了能提供的道路(Road)级别的导航信息外，还能够提供车道(Lane)级别的导航信息。
无论是在信息的丰富度还是信息的精度方面，都是远远高于传统导航地图的。 目前市面上提供高精度地图的厂商有：tomtom、here、百度、高德等。 
高精度地图流行的格式有很多种，有的厂商直接基于rndf地图增加属性来制作高精度地图，也有厂商使用osm格式增加属性来制作高精度地图。 
对于ADAS系统，则有ADASIS定义了地图的数据模型及传输方式，以CAN作为传输通道。 OpenDRIVE是一种开放的文件格式, 用于路网的逻辑描述，常用于高精度地图的制作，百度Apollo则使用基于OpenDRIVE格式改进过的高精度地图。

本文主要对OpenDRIVE文件格式进行简述，详情可参考：http://www.opendrive.org/docs/OpenDRIVEFormatSpecDelta_1.5M_vs_1.4H.pdf


2. // OpenDRIVE 
		|-header 
		| |-geoReference 
		| |-offset 
		|-road  
		| |-link 
		| | |-predecessor 
		| | |-successor 
		| | |-neighbor 
		| |-type 
		| | |-speed 
		| |-planView 
		| | |-geometry | | | |-line 
		| | | |-spiral 
		| | | |-arc 
		| | | |-poly3 
		| | | |-paramPoly3 
		| |-elevationProfile 
		| | |-elevation 
		| |-lateralProfile 
		| | |-superelevation 
		| | |-crossfall 
		| | |-shape 
		| |-lanes 
		| | |-laneOffset 
		| | |-laneSection 
		| | | |-left 
		| | | | |-lane 
		| | | | | |-link 
		| | | | | | |-predecessor 
		| | | | | | |-successor 
		| | | | | |-width 
		| | | | | |-border 
		| | | | | |-roadMark 
		| | | | | | | -sway 
		| | | | | | | -type 
		| | | | | | | | -line 
		| | | | | | | -explicit 
		| | | | | | | | -line 
		| | | | | |-material 
		| | | | | |-visibility 
		| | | | | |-speed 
		| | | | | |-access 
		| | | | | |-height 
		| | | | | |-rule 
		| | | |-center 
		| | | | |-lane 
		| | | | | |-link 
		| | | | | | |-predecessor 
		| | | | | | |-successor
		| | | | | | |-predecessor 
		| | | | | | |-successor 
		| | | | | |-roadMark 
		| | | | | | | -sway 
		| | | | | | | -type 
		| | | | | | | | -line 
		| | | | | | | -explicit 
		| | | | | | | | -line 
		| | | |-right 
		| | | | |-lane 
		| | | | | |-link 
		| | | | | | |-predecessor 
		| | | | | | |-successor 
		| | | | | |-width 
		| | | | | |-border 
		| | | | | |-roadMark 
		| | | | | | | -sway 
		| | | | | | | -type 
		| | | | | | | | -line 
		| | | | | | | -explicit 
		| | | | | | | | -line 
		| | | | | |-material 
		| | | | | |-visibility 
		| | | | | |-speed 
		| | | | | |-access 
		| | | | | |-height 
		| | | | | |-rule 
		| |-objects 
		| | |-object 
		| | | |-repeat 
		| | | |-outlines 
		| | | | |-outline 
		| | | | | |-cornerRoad 
		| | | | | |-cornerLocal 
		| | | |-material 
		| | | |-validity 
		| | | |-parkingSpace 
		| | | |-markings 
		| | | | |-marking 
		| | | | | |-cornerReference 
		| | | |-borders 
		| | | | |-border 
		| | | | | |-cornerReference 
		| | |-objectReference 
		| | | |-validity | | |-tunnel 
		| | | |-validity | | |-bridge 
		| | | |-validity 
		| |-signals 
		| | |-signal 
		| | | |-validity 
		| | | |-dependency 
		| | | |-reference 
		| | | |-positionRoad 
		| | | |-positionInertial 
		| | |-signalReference 
		| | | |-validity 
		| |-surface 
		| | |-CRG 
		| |-railroad 
		| | |-switch 
		| | | |-mainTrack 
		| | | |-sideTrack 
		| | | |-partner 
		|-controller 
		| |-control 
		|-junction 
		| |-connection 
		| | |-predecessor 
		| | |-successor 
		| | |-laneLink | |-priority 
		| |-controller 
		| |-surface 
		| | |-CRG 
		|-junctionGroup 
		| |-junctionReference 
		|-station 
		| |-platform 
		| | |-segment 
	


3. 
  总之，对于一个road来说，先确定reference line，有了reference line的几何形状和位置，然后再确定reference line左右的车道lane,车道lane又有实线和虚线等属性；road 和road之间通过普通连接和Junction进行连接，同时还要将road中的相关车道进行连接

  |-Header
	|-Road
	|	|-Type
	|	|-Link
	|	|-Plan View/Chord line
	|	|	|- Geometry
	|	|		|-Line
	|	|		|-Spiral
	|	|		|-Arc
	|	|		|-Cubic polynomial
	|	|
	|	|-Elevation
	|	|-Superelevation
	|	|-Crossfall
	|	|-Lane Section
	|	|	|-Left lanes
	|	|	|	|-Lane
	|	|	|		|-Link
	|	|	|		|-Width
	|	|	|		|-Road Mark
	|	|	|		|-Matrial
	|	|	|		|-Visibility
	|	|	|		|-Speed limit
	|	|	|		|-Access
	|	|	|		|-Height
	|	|	|
	|	|	|-Center lanes
	|	|	|-Right lanes
	|	|
	|	|-Ojects
	|	|-Signals
	|
	|
	|-Junction
		|-Connection
		|	|-Lane links
		|
		|-Priority
		|-Controller
	
	
	
		
		
4.  // geoReference 坐标系
	<header revMajor="1" revMinor="4" name="OpenDRIVE TestFile" version="1" date="Tue Aug 15 16:21:00 2017" north="5.4355531085039526e+06" south="5.4143611839699000e+06" east="3.2681018217470363e+07" west="3.2670519445542485e+07" vendor="AUTONAVI">
			<geoReference originLat="3.2675915701523371e+07" originLong="5.4273604273430230e+06" originAlt="0.0000000000000000e+00" originHdg="0.0000000000000000e+00">
				<![CDATA[PROJCS["WGS 84 / UTM zone 32N",GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563,AUTHORITY["EPSG","7030"]],AUTHORITY["EPSG","6326"]],PRIMEM["Greenwich",0,AUTHORITY["EPSG","8901"]],UNIT["degree",0.01745329251994328,AUTHORITY["EPSG","9122"]],AUTHORITY["EPSG","4326"]],UNIT["metre",1,AUTHORITY["EPSG","9001"]],PROJECTION["Transverse_Mercator"],PARAMETER["latitude_of_origin",0],PARAMETER["central_meridian",117],PARAMETER["scale_factor",0.9996],PARAMETER["false_easting",500000],PARAMETER["false_northing",0],AUTHORITY["EPSG","32650"],AXIS["Easting",EAST],AXIS["Northing",NORTH]]]]>
			</geoReference>
	</header>




5. // reference line
	reference line是路网结构中一个很重要的概念，绘制地图的时候先是画reference line，reference line包含xy位置坐标、路的形状属性，然后在reference line基础上再去画其他其他元素。
	下图是OpenDRIVE中路网结构中的一个road，该road有三部分组成，蓝色的reference line，车道lane，车道lane的其他feature(限速等)。
	整个地图路网由很多的road构成，而每个road中都会包含reference line，就是一条线，它没有宽度
	
	而在OpenDRIVE数据中，reference line体现在planView元素下的geometry,顾名思义，俯视图下的road几何形状，该road是条直线，螺旋线等等。	
		
		
	<planView>
      <geometry hdg="3.3217917505880004" length="4.7098111708266241" s="0" x="-86.484407620095055" y="-14.570421678104463">
        <paramPoly3 aU="0" aV="0" bU="4.7095295283576153" bV="6.6613381477509392E-16" cU="0.0026919779426197721" cV="-0.038957641607217486" dU="-0.0029213483020198439" dV="0.085437748644380029" pRange="normalized" />
      </geometry>
      <geometry hdg="3.3596810245363811" length="0.85448778624532851" s="4.7098111708266241" x="-91.109124508574951" y="-15.460175621552397">
        <paramPoly3 aU="0" aV="0" bU="0.85446559372837283" bV="4.163336342344337E-17" cU="0.00016012697394696129" cV="-0.0020006071452756957" dU="-0.0001955235515389564" dV="-0.0057768643261346259" pRange="normalized" />
      </geometry>
    </planView>
		
	一个road的并不是只有一根reference line，因为假如一个road长度为100米，有可能这100米有些地方是直路，有些地方是拐弯的曲线，每一条都是一个geometry标签，通过s(起始位置)和长度进行连接(后一个s是前一个的length)。
	而属性中的x，y，hdg分别是投影坐标系xy下的起始点位置以及起始点的角度(定义了曲线方程以及起始点坐标和长度，曲线肯定就能画出来了)。


6. // 车道lane
		
	一个road中包含了很多的车道lane(lanes)，而车道(lane)本身有宽度(width)，以及虚线、实线等属性参数(roadMark)。结合这些参数，我们就能在reference line的基础上将车道画出来。
	在lanes下还有个laneSection车道横截面的概念，一个road包含了数个laneSection，每个laneSection中又包含了车道lane，在一个laneSection中车道lane是顺着reference line分为left,center,right。
	reference line是center，没有宽度width，只是一条线。
	left的lane的id为正，right为负。上图中坐边定义了5条车道，1,2,3,-1,-2，而右边多了一条-3。

	此外，在lane元素中，width元素定义了车道的宽度，都是基于曲线进行拟合的。roadMark元素定义了车道线的属性，OpenDrive中规定的车道线的属性如下所示，有实线，虚线等。


			
	<lanes>
				<laneSection s="0.0000000000000000e+00">
					<center>
						<lane id="0" type="driving" level="false">
							<link />
							<roadMark sOffset="0.0000000000000000e+00" type="solid" weight="standard" color="standard" material="standard" LDM="none" width="2.9999999999999999e-01" laneChange="both" />
						</lane>
					</center>
					<right>
						<lane id="-1" type="driving" level="false">
							<link>
								<predecessor id="-1" />
								<successor id="-1" />
							</link>
							<width sOffset="0.0000000000000000e+00" a="3.8890850467340541e+00" b="-1.4514389448175911e-03" c="1.0899364495936138e-04" d="-1.3397356888919692e-06" />
							<width sOffset="7.6500000000000000e+01" a="3.8161122099921028e+00" b="1.6531839124687595e-03" c="-3.0234314157904548e-06" d="-2.9791355887866248e-08" />
							<roadMark sOffset="0.0000000000000000e+00" type="broken" weight="standard" color="standard" material="standard" LDM="none" width="1.4999999999999999e-01" laneChange="both" />
						</lane>
					</right>
				</laneSection>
	 </lanes>

			
	
7. // road连接	
		通过上面的介绍，已经画完了一个road，而前面提到OpenDrive地图是由多个road组成，下面介绍如何将这些road连接起来

		road之间的连接定义了两种(每个road有唯一的ID)，一种是有明确的连接关系，例如前后只有一条road，那么通过successor/predecessor进行连接(由于road中又包含了很多lanes车道，所以需要将车道的连接关系也表示清楚)
		
	<lanes>
      <laneSection s="0" singleSide="false">
        <center>
          <lane id="0" level="false" type="none" />
        </center>
        <right>
          <lane id="-1" level="false" type="driving">
            <link>
              <predecessor id="-1" />
              <successor id="-1" />
            </link>
            <width a="3.4023015592114376" b="0.0034711086799576892" c="0.0018714533638347648" d="-0.00022422152023268626" sOffset="0" />
            <height inner="0" outer="0" sOffset="0" />
            <height inner="0" outer="0" sOffset="5.5642989780601795" />
          </lane>
          <lane id="-2" level="false" type="driving">
            <link>
              <predecessor id="-2" />
              <successor id="-2" />
            </link>
            <width a="3.3806637900074357" b="-0.00015236307390843875" c="-8.2146775995970344E-05" d="9.842123427690379E-06" sOffset="0" />
            <height inner="0" outer="0" sOffset="0" />
            <height inner="0" outer="0" sOffset="5.5642989780601795" />
          </lane>
          <lane id="-3" level="false" type="parking">
            <link>
              <predecessor id="-3" />   // the lanes connection
              <successor id="-3" />
            </link>
            <width a="6.8568044755651574" b="0.032978589237821494" c="0.038786276457064874" d="-0.011148490913612676" sOffset="0" />
            <width a="6.9887188327646692" b="0.10868360778237368" c="-0.1464201549381185" d="0.12318940558111113" sOffset="2.00000000376344" />
            <width a="7.074171691538929" b="0.1854115154892759" c="0.20752574227039694" d="-0.107567085380266" sOffset="3.00000000564516" />
            <width a="7.359541864441006" b="0.27776174345581589" c="0.11340704563739842" d="-0.12101547430463838" sOffset="4.00000000752688" />
            <width a="7.6296951794959007" b="0.25080572357185921" c="-0.78759047408866323" d="0.67989350656097736" sOffset="5.0000000094086" />
            <height inner="0" outer="0" sOffset="0" />
            <height inner="0" outer="0" sOffset="5.5642989780601795" />
          </lane>
        </right>
      </laneSection>
    </lanes>
			
			
			
			
			
			
		 <road id="387" junction="541" length="18.877464310495206" name="Road (15934)">  // junction is the connection between different roads
			<link>
			  <predecessor contactPoint="end" elementId="355" elementType="road" />
			  <successor contactPoint="start" elementId="3" elementType="road" />
			</link>
			<type s="0" type="unknown">
			  <speed max="undefined" unit="km/h" />
			</type>
			<planView>
			  <geometry hdg="4.9081279100280018" length="8.1826222019938335" s="0" x="7.4377043484207963" y="-128.47208083611821">
				<paramPoly3 aU="0" aV="0" bU="8.026490790618082" bV="0" cU="0.054320972117686495" cV="-4.0539383379687095" dU="-0.667927288457749" dV="0.976001011952258" pRange="normalized" />
			  </geometry>
			  <geometry hdg="4.2066501518664738" length="10.694842108501373" s="8.1826222019938335" x="5.8602849634345553" y="-136.34204287514652">
				<paramPoly3 aU="0" aV="0" bU="10.437937300742339" bV="2.4424906541753444E-15" cU="1.1882525172878609" cV="-3.3963526747469146" dU="-1.8343271402624364" dV="-0.21888734411305677" pRange="normalized" />
			  </geometry>
			</planView>
			<elevationProfile>
			  <elevation a="21.473354845885375" b="-0.000317862425700696" c="-5.0483851567221372E-05" d="1.7817766171013202E-06" s="0" />
			</elevationProfile>
			<lateralProfile>
			  <superelevation a="0" b="0" c="0" d="0" s="0" />
			  <crossfall a="0" b="0" c="0" d="0" s="0" side="left" />
			  <crossfall a="0.00054299602844392009" b="-1.43383690970215E-05" c="-2.277262232597187E-06" d="8.03736733844483E-08" s="0" side="right" />
			</lateralProfile>
			<lanes>
			  <laneSection s="0" singleSide="false">
				<center>
				  <lane id="0" level="false" type="none" />
				</center>
				<right>
				  <lane id="-1" level="false" type="driving">
					<link>
					  <predecessor id="-4" />
					  <successor id="-2" />
					</link>
					<width a="4.8235445004964221" b="0.03909024094240153" c="0.028710225129432814" d="-0.0052267150044249047" sOffset="0" />
					<width a="5.0580859286291622" b="0.0702302948157919" c="0.0029372337920214138" d="-0.0018624322545796807" sOffset="2.9999997758817964" />
					<width a="5.2449262379113986" b="0.056352049336472837" c="-0.025397676957451227" d="0.0034318099718745219" sOffset="5.9999995517635929" />
					<width a="5.2834941091356935" b="-0.0040569373944415212" c="-0.002716691274467603" d="-0.0044768737449461995" sOffset="7.9999994023514578" />
					<width a="5.2286984895459439" b="-0.03432308929653085" c="-0.015891419589759207" d="0.0026838435406608225" sOffset="9.99999925293932" />
					<width a="5.0089094152765909" b="-0.13051986325336865" c="0.20244400593245074" d="-0.1131747802827778" sOffset="13.99999895411505" />
					<width a="4.9676587825404583" b="-0.065156171755333711" c="-0.078687350063013445" d="0.05478181643213078" sOffset="14.999998879408981" />
					<width a="4.878597081501046" b="-0.020221089678419336" c="0.0058090785749070513" d="-0.00072284744858916959" sOffset="15.999998804702916" />
					<height inner="0" outer="0" sOffset="0" />
					<height inner="0" outer="0" sOffset="18.877461491524386" />
				  </lane>
				</right>
			  </laneSection>
			</lanes>
		  </road>
			
	如果前后的连接关系不是很明确，就需要一个junctions
			
		
		
		
		
		
		
		
		
		
