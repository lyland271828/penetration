function [activate_plot,...
time_interval_plot,...
activate_trajectory,...
follow_agent,...
activate_save_figure,...
activate_save_video,...
dim_visual,...
time_interval_trajectory,...
video_speed,...
x_range,...
y_range,...
z_range,...
legend_name,...
font_size,...
font_size_sub,...
marker_size,...
background_color,...
activate_BD_1,...
len_arm,...
cmap_terrain,...
cmap_traj,...
T_end,...
rader_pos,...
rader_Rmax] = visual_module_parameters()
%VISUAL_MODULE_PARAMETERS 
% Automatically generated by read_parameter_xml.m
% Every time read_parameter_xml.m is run, this function will be generated
activate_plot = 1.000000000000;
time_interval_plot = 1.000000000000;
activate_trajectory = 1.000000000000;
follow_agent = 0.000000000000;
activate_save_figure = 0.000000000000;
activate_save_video = 0.000000000000;
dim_visual = 3.000000000000;
time_interval_trajectory = 40.000000000000;
video_speed = 10.000000000000;
x_range = [-506.000000000000
561.000000000000];
y_range = [-510.000000000000
552.000000000000];
z_range = [0.000000000000
252.000000000000];
legend_name = ['$', '\', 'p', 'h', 'i', '^', '{', 'c', 'o', 'r', 'r', '}', '$', '|', '|', '|', '$', '\', 'p', 'h', 'i', '^', '{', 'v', 'e', 'l', '}', '$', '|', '|', '|', '$', '\', 'p', 'h', 'i', '^', '{', 'c', 'o', 'l', 'l', '}', '$', '|', '|', '|', '$', '\', 'p', 'h', 'i', '^', '{', 'w', 'a', 'l', 'l', '}', '$', '|', '|', '|', '$', '\', 'p', 'h', 'i', '^', '{', 'g', 'r', 'o', 'u', 'p', '}', '$'];
font_size = 10.000000000000;
font_size_sub = 8.000000000000;
marker_size = 10.000000000000;
background_color = [1.000000000000, 1.000000000000, 1.000000000000];
activate_BD_1 = 0.000000000000;
len_arm = 0.500000000000;
cmap_terrain = [0.031372549000, 0.188235294000, 0.419607843000
0.031372549000, 0.192295270941, 0.425636293604
0.031372549000, 0.196355247882, 0.431664744208
0.031372549000, 0.200415224824, 0.437693194812
0.031372549000, 0.204475201765, 0.443721645416
0.031372549000, 0.208535178706, 0.449750096020
0.031372549000, 0.212595155647, 0.455778546624
0.031372549000, 0.216655132588, 0.461806997227
0.031372549000, 0.220715109529, 0.467835447831
0.031372549000, 0.224775086471, 0.473863898435
0.031372549000, 0.228835063412, 0.479892349039
0.031372549000, 0.232895040353, 0.485920799643
0.031372549000, 0.236955017294, 0.491949250247
0.031372549000, 0.241014994235, 0.497977700851
0.031372549000, 0.245074971176, 0.504006151455
0.031372549000, 0.249134948118, 0.510034602059
0.031372549000, 0.253194925059, 0.516063052663
0.031372549000, 0.257254902000, 0.522091503267
0.031372549000, 0.261314878941, 0.528119953871
0.031372549000, 0.265374855882, 0.534148404475
0.031372549000, 0.269434832824, 0.540176855078
0.031372549000, 0.273494809765, 0.546205305682
0.031372549000, 0.277554786706, 0.552233756286
0.031372549000, 0.281614763647, 0.558262206890
0.031372549000, 0.285674740588, 0.564290657494
0.031372549000, 0.289734717529, 0.570319108098
0.031372549000, 0.293794694471, 0.576347558702
0.031372549000, 0.297854671412, 0.582376009306
0.031372549000, 0.301914648353, 0.588404459910
0.031372549000, 0.305974625294, 0.594432910514
0.031372549000, 0.310034602235, 0.600461361118
0.031372549000, 0.314094579176, 0.606489811722
0.031757016514, 0.318139177416, 0.612149173514
0.034832756624, 0.322076124741, 0.615224913624
0.037908496733, 0.326013072067, 0.618300653733
0.040984236843, 0.329950019392, 0.621376393843
0.044059976953, 0.333886966718, 0.624452133953
0.047135717063, 0.337823914043, 0.627527874063
0.050211457173, 0.341760861369, 0.630603614173
0.053287197282, 0.345697808694, 0.633679354282
0.056362937392, 0.349634756020, 0.636755094392
0.059438677502, 0.353571703345, 0.639830834502
0.062514417612, 0.357508650671, 0.642906574612
0.065590157722, 0.361445597996, 0.645982314722
0.068665897831, 0.365382545322, 0.649058054831
0.071741637941, 0.369319492647, 0.652133794941
0.074817378051, 0.373256439973, 0.655209535051
0.077893118161, 0.377193387298, 0.658285275161
0.080968858271, 0.381130334624, 0.661361015271
0.084044598380, 0.385067281949, 0.664436755380
0.087120338490, 0.389004229275, 0.667512495490
0.090196078600, 0.392941176600, 0.670588235600
0.093271818710, 0.396878123925, 0.673663975710
0.096347558820, 0.400815071251, 0.676739715820
0.099423298929, 0.404752018576, 0.679815455929
0.102499039039, 0.408688965902, 0.682891196039
0.105574779149, 0.412625913227, 0.685966936149
0.108650519259, 0.416562860553, 0.689042676259
0.111726259369, 0.420499807878, 0.692118416369
0.114801999478, 0.424436755204, 0.695194156478
0.117877739588, 0.428373702529, 0.698269896588
0.120953479698, 0.432310649855, 0.701345636698
0.124029219808, 0.436247597180, 0.704421376808
0.127104959918, 0.440184544506, 0.707497116918
0.130426759227, 0.444152249235, 0.710326797812
0.134486736137, 0.448212226176, 0.712418301059
0.138546713047, 0.452272203118, 0.714509804306
0.142606689957, 0.456332180059, 0.716601307553
0.146666666867, 0.460392157000, 0.718692810800
0.150726643776, 0.464452133941, 0.720784314047
0.154786620686, 0.468512110882, 0.722875817294
0.158846597596, 0.472572087824, 0.724967320541
0.162906574506, 0.476632064765, 0.727058823788
0.166966551416, 0.480692041706, 0.729150327035
0.171026528325, 0.484752018647, 0.731241830282
0.175086505235, 0.488811995588, 0.733333333529
0.179146482145, 0.492871972529, 0.735424836776
0.183206459055, 0.496931949471, 0.737516340024
0.187266435965, 0.500991926412, 0.739607843271
0.191326412875, 0.505051903353, 0.741699346518
0.195386389784, 0.509111880294, 0.743790849765
0.199446366694, 0.513171857235, 0.745882353012
0.203506343604, 0.517231834176, 0.747973856259
0.207566320514, 0.521291811118, 0.750065359506
0.211626297424, 0.525351788059, 0.752156862753
0.215686274333, 0.529411765000, 0.754248366000
0.219746251243, 0.533471741941, 0.756339869247
0.223806228153, 0.537531718882, 0.758431372494
0.227866205063, 0.541591695824, 0.760522875741
0.231926181973, 0.545651672765, 0.762614378988
0.235986158882, 0.549711649706, 0.764705882235
0.240046135792, 0.553771626647, 0.766797385482
0.244106112702, 0.557831603588, 0.768888888729
0.248166089612, 0.561891580529, 0.770980391976
0.252226066522, 0.565951557471, 0.773071895224
0.256286043431, 0.570011534412, 0.775163398471
0.260715109165, 0.573840830835, 0.777208765624
0.265759322937, 0.577285659729, 0.779177239286
0.270803536710, 0.580730488624, 0.781145712949
0.275847750482, 0.584175317518, 0.783114186612
0.280891964255, 0.587620146412, 0.785082660275
0.285936178027, 0.591064975306, 0.787051133937
0.290980391800, 0.594509804200, 0.789019607600
0.296024605573, 0.597954633094, 0.790988081263
0.301068819345, 0.601399461988, 0.792956554925
0.306113033118, 0.604844290882, 0.794925028588
0.311157246890, 0.608289119776, 0.796893502251
0.316201460663, 0.611733948671, 0.798861975914
0.321245674435, 0.615178777565, 0.800830449576
0.326289888208, 0.618623606459, 0.802798923239
0.331334101980, 0.622068435353, 0.804767396902
0.336378315753, 0.625513264247, 0.806735870565
0.341422529525, 0.628958093141, 0.808704344227
0.346466743298, 0.632402922035, 0.810672817890
0.351510957071, 0.635847750929, 0.812641291553
0.356555170843, 0.639292579824, 0.814609765216
0.361599384616, 0.642737408718, 0.816578238878
0.366643598388, 0.646182237612, 0.818546712541
0.371687812161, 0.649627066506, 0.820515186204
0.376732025933, 0.653071895400, 0.822483659867
0.381776239706, 0.656516724294, 0.824452133529
0.386820453478, 0.659961553188, 0.826420607192
0.391864667251, 0.663406382082, 0.828389080855
0.396908881024, 0.666851210976, 0.830357554518
0.401953094796, 0.670296039871, 0.832326028180
0.406997308569, 0.673740868765, 0.834294501843
0.412041522341, 0.677185697659, 0.836262975506
0.417085736114, 0.680630526553, 0.838231449169
0.422745097902, 0.684075355463, 0.839892348824
0.429019607706, 0.687520184388, 0.841245674471
0.435294117510, 0.690965013314, 0.842599000118
0.441568627314, 0.694409842239, 0.843952325765
0.447843137118, 0.697854671165, 0.845305651412
0.454117646922, 0.701299500090, 0.846658977059
0.460392156725, 0.704744329016, 0.848012302706
0.466666666529, 0.708189157941, 0.849365628353
0.472941176333, 0.711633986867, 0.850718954000
0.479215686137, 0.715078815792, 0.852072279647
0.485490195941, 0.718523644718, 0.853425605294
0.491764705745, 0.721968473643, 0.854778930941
0.498039215549, 0.725413302569, 0.856132256588
0.504313725353, 0.728858131494, 0.857485582235
0.510588235157, 0.732302960420, 0.858838907882
0.516862744961, 0.735747789345, 0.860192233529
0.523137254765, 0.739192618271, 0.861545559176
0.529411764569, 0.742637447196, 0.862898884824
0.535686274373, 0.746082276122, 0.864252210471
0.541960784176, 0.749527105047, 0.865605536118
0.548235293980, 0.752971933973, 0.866958861765
0.554509803784, 0.756416762898, 0.868312187412
0.560784313588, 0.759861591824, 0.869665513059
0.567058823392, 0.763306420749, 0.871018838706
0.573333333196, 0.766751249675, 0.872372164353
0.579607843000, 0.770196078600, 0.873725490000
0.585882352804, 0.773640907525, 0.875078815647
0.592156862608, 0.777085736451, 0.876432141294
0.598431372412, 0.780530565376, 0.877785466941
0.604705882216, 0.783975394302, 0.879138792588
0.610980392020, 0.787420223227, 0.880492118235
0.617254901824, 0.790865052153, 0.881845443882
0.622683583098, 0.793464052529, 0.883429450039
0.627604767255, 0.795555555776, 0.885151864502
0.632525951412, 0.797647059024, 0.886874278965
0.637447135569, 0.799738562271, 0.888596693427
0.642368319725, 0.801830065518, 0.890319107890
0.647289503882, 0.803921568765, 0.892041522353
0.652210688039, 0.806013072012, 0.893763936816
0.657131872196, 0.808104575259, 0.895486351278
0.662053056353, 0.810196078506, 0.897208765741
0.666974240510, 0.812287581753, 0.898931180204
0.671895424667, 0.814379085000, 0.900653594667
0.676816608824, 0.816470588247, 0.902376009129
0.681737792980, 0.818562091494, 0.904098423592
0.686658977137, 0.820653594741, 0.905820838055
0.691580161294, 0.822745097988, 0.907543252518
0.696501345451, 0.824836601235, 0.909265666980
0.701422529608, 0.826928104482, 0.910988081443
0.706343713765, 0.829019607729, 0.912710495906
0.711264897922, 0.831111110976, 0.914432910369
0.716186082078, 0.833202614224, 0.916155324831
0.721107266235, 0.835294117471, 0.917877739294
0.726028450392, 0.837385620718, 0.919600153757
0.730949634549, 0.839477123965, 0.921322568220
0.735870818706, 0.841568627212, 0.923044982682
0.740792002863, 0.843660130459, 0.924767397145
0.745713187020, 0.845751633706, 0.926489811608
0.750634371176, 0.847843136953, 0.928212226071
0.755555555333, 0.849934640200, 0.929934640533
0.760476739490, 0.852026143447, 0.931657054996
0.765397923647, 0.854117646694, 0.933379469459
0.770319107804, 0.856209149941, 0.935101883922
0.775240291961, 0.858300653188, 0.936824298384
0.778685120871, 0.860299884247, 0.937993079624
0.781637831365, 0.862268357910, 0.938977316455
0.784590541859, 0.864236831573, 0.939961553286
0.787543252353, 0.866205305235, 0.940945790118
0.790495962847, 0.868173778898, 0.941930026949
0.793448673341, 0.870142252561, 0.942914263780
0.796401383835, 0.872110726224, 0.943898500612
0.799354094329, 0.874079199886, 0.944882737443
0.802306804824, 0.876047673549, 0.945866974275
0.805259515318, 0.878016147212, 0.946851211106
0.808212225812, 0.879984620875, 0.947835447937
0.811164936306, 0.881953094537, 0.948819684769
0.814117646800, 0.883921568200, 0.949803921600
0.817070357294, 0.885890041863, 0.950788158431
0.820023067788, 0.887858515525, 0.951772395263
0.822975778282, 0.889826989188, 0.952756632094
0.825928488776, 0.891795462851, 0.953740868925
0.828881199271, 0.893763936514, 0.954725105757
0.831833909765, 0.895732410176, 0.955709342588
0.834786620259, 0.897700883839, 0.956693579420
0.837739330753, 0.899669357502, 0.957677816251
0.840692041247, 0.901637831165, 0.958662053082
0.843644751741, 0.903606304827, 0.959646289914
0.846597462235, 0.905574778490, 0.960630526745
0.849550172729, 0.907543252153, 0.961614763576
0.852502883224, 0.909511725816, 0.962599000408
0.855455593718, 0.911480199478, 0.963583237239
0.858408304212, 0.913448673141, 0.964567474071
0.861361014706, 0.915417146804, 0.965551710902
0.864313725200, 0.917385620467, 0.966535947733
0.867266435694, 0.919354094129, 0.967520184565
0.870219146188, 0.921322567792, 0.968504421396
0.873279507596, 0.923291041455, 0.969488658227
0.876355247706, 0.925259515118, 0.970472895059
0.879430987816, 0.927227988780, 0.971457131890
0.882506727925, 0.929196462443, 0.972441368722
0.885582468035, 0.931164936106, 0.973425605553
0.888658208145, 0.933133409769, 0.974409842384
0.891733948255, 0.935101883431, 0.975394079216
0.894809688365, 0.937070357094, 0.976378316047
0.897885428475, 0.939038830757, 0.977362552878
0.900961168584, 0.941007304420, 0.978346789710
0.904036908694, 0.942975778082, 0.979331026541
0.907112648804, 0.944944251745, 0.980315263373
0.910188388914, 0.946912725408, 0.981299500204
0.913264129024, 0.948881199071, 0.982283737035
0.916339869133, 0.950849672733, 0.983267973867
0.919415609243, 0.952818146396, 0.984252210698
0.922491349353, 0.954786620059, 0.985236447529
0.925567089463, 0.956755093722, 0.986220684361
0.928642829573, 0.958723567384, 0.987204921192
0.931718569682, 0.960692041047, 0.988189158024
0.934794309792, 0.962660514710, 0.989173394855
0.937870049902, 0.964628988373, 0.990157631686
0.940945790012, 0.966597462035, 0.991141868518
0.944021530122, 0.968565935698, 0.992126105349
0.947097270231, 0.970534409361, 0.993110342180
0.950173010341, 0.972502883024, 0.994094579012
0.953248750451, 0.974471356686, 0.995078815843
0.956324490561, 0.976439830349, 0.996063052675
0.959400230671, 0.978408304012, 0.997047289506
0.962475970780, 0.980376777675, 0.998031526337
0.965551710890, 0.982345251337, 0.999015763169
0.968627451000, 0.984313725000, 1.000000000000];
cmap_traj = [1.000000000000, 0.000000000000, 0.000000000000
1.000000000000, 0.023437500000, 0.000000000000
1.000000000000, 0.046875000000, 0.000000000000
1.000000000000, 0.070312500000, 0.000000000000
1.000000000000, 0.093750000000, 0.000000000000
1.000000000000, 0.117187500000, 0.000000000000
1.000000000000, 0.140625000000, 0.000000000000
1.000000000000, 0.164062500000, 0.000000000000
1.000000000000, 0.187500000000, 0.000000000000
1.000000000000, 0.210937500000, 0.000000000000
1.000000000000, 0.234375000000, 0.000000000000
1.000000000000, 0.257812500000, 0.000000000000
1.000000000000, 0.281250000000, 0.000000000000
1.000000000000, 0.304687500000, 0.000000000000
1.000000000000, 0.328125000000, 0.000000000000
1.000000000000, 0.351562500000, 0.000000000000
1.000000000000, 0.375000000000, 0.000000000000
1.000000000000, 0.398437500000, 0.000000000000
1.000000000000, 0.421875000000, 0.000000000000
1.000000000000, 0.445312500000, 0.000000000000
1.000000000000, 0.468750000000, 0.000000000000
1.000000000000, 0.492187500000, 0.000000000000
1.000000000000, 0.515625000000, 0.000000000000
1.000000000000, 0.539062500000, 0.000000000000
1.000000000000, 0.562500000000, 0.000000000000
1.000000000000, 0.585937500000, 0.000000000000
1.000000000000, 0.609375000000, 0.000000000000
1.000000000000, 0.632812500000, 0.000000000000
1.000000000000, 0.656250000000, 0.000000000000
1.000000000000, 0.679687500000, 0.000000000000
1.000000000000, 0.703125000000, 0.000000000000
1.000000000000, 0.726562500000, 0.000000000000
1.000000000000, 0.750000000000, 0.000000000000
1.000000000000, 0.773437500000, 0.000000000000
1.000000000000, 0.796875000000, 0.000000000000
1.000000000000, 0.820312500000, 0.000000000000
1.000000000000, 0.843750000000, 0.000000000000
1.000000000000, 0.867187500000, 0.000000000000
1.000000000000, 0.890625000000, 0.000000000000
1.000000000000, 0.914062500000, 0.000000000000
1.000000000000, 0.937500000000, 0.000000000000
1.000000000000, 0.960937500000, 0.000000000000
1.000000000000, 0.984375000000, 0.000000000000
0.992187500000, 1.000000000000, 0.000000000000
0.968750000000, 1.000000000000, 0.000000000000
0.945312500000, 1.000000000000, 0.000000000000
0.921875000000, 1.000000000000, 0.000000000000
0.898437500000, 1.000000000000, 0.000000000000
0.875000000000, 1.000000000000, 0.000000000000
0.851562500000, 1.000000000000, 0.000000000000
0.828125000000, 1.000000000000, 0.000000000000
0.804687500000, 1.000000000000, 0.000000000000
0.781250000000, 1.000000000000, 0.000000000000
0.757812500000, 1.000000000000, 0.000000000000
0.734375000000, 1.000000000000, 0.000000000000
0.710937500000, 1.000000000000, 0.000000000000
0.687500000000, 1.000000000000, 0.000000000000
0.664062500000, 1.000000000000, 0.000000000000
0.640625000000, 1.000000000000, 0.000000000000
0.617187500000, 1.000000000000, 0.000000000000
0.593750000000, 1.000000000000, 0.000000000000
0.570312500000, 1.000000000000, 0.000000000000
0.546875000000, 1.000000000000, 0.000000000000
0.523437500000, 1.000000000000, 0.000000000000
0.500000000000, 1.000000000000, 0.000000000000
0.476562500000, 1.000000000000, 0.000000000000
0.453125000000, 1.000000000000, 0.000000000000
0.429687500000, 1.000000000000, 0.000000000000
0.406250000000, 1.000000000000, 0.000000000000
0.382812500000, 1.000000000000, 0.000000000000
0.359375000000, 1.000000000000, 0.000000000000
0.335937500000, 1.000000000000, 0.000000000000
0.312500000000, 1.000000000000, 0.000000000000
0.289062500000, 1.000000000000, 0.000000000000
0.265625000000, 1.000000000000, 0.000000000000
0.242187500000, 1.000000000000, 0.000000000000
0.218750000000, 1.000000000000, 0.000000000000
0.195312500000, 1.000000000000, 0.000000000000
0.171875000000, 1.000000000000, 0.000000000000
0.148437500000, 1.000000000000, 0.000000000000
0.125000000000, 1.000000000000, 0.000000000000
0.101562500000, 1.000000000000, 0.000000000000
0.078125000000, 1.000000000000, 0.000000000000
0.054687500000, 1.000000000000, 0.000000000000
0.031250000000, 1.000000000000, 0.000000000000
0.007812500000, 1.000000000000, 0.000000000000
0.000000000000, 1.000000000000, 0.015625000000
0.000000000000, 1.000000000000, 0.039062500000
0.000000000000, 1.000000000000, 0.062500000000
0.000000000000, 1.000000000000, 0.085937500000
0.000000000000, 1.000000000000, 0.109375000000
0.000000000000, 1.000000000000, 0.132812500000
0.000000000000, 1.000000000000, 0.156250000000
0.000000000000, 1.000000000000, 0.179687500000
0.000000000000, 1.000000000000, 0.203125000000
0.000000000000, 1.000000000000, 0.226562500000
0.000000000000, 1.000000000000, 0.250000000000
0.000000000000, 1.000000000000, 0.273437500000
0.000000000000, 1.000000000000, 0.296875000000
0.000000000000, 1.000000000000, 0.320312500000
0.000000000000, 1.000000000000, 0.343750000000
0.000000000000, 1.000000000000, 0.367187500000
0.000000000000, 1.000000000000, 0.390625000000
0.000000000000, 1.000000000000, 0.414062500000
0.000000000000, 1.000000000000, 0.437500000000
0.000000000000, 1.000000000000, 0.460937500000
0.000000000000, 1.000000000000, 0.484375000000
0.000000000000, 1.000000000000, 0.507812500000
0.000000000000, 1.000000000000, 0.531250000000
0.000000000000, 1.000000000000, 0.554687500000
0.000000000000, 1.000000000000, 0.578125000000
0.000000000000, 1.000000000000, 0.601562500000
0.000000000000, 1.000000000000, 0.625000000000
0.000000000000, 1.000000000000, 0.648437500000
0.000000000000, 1.000000000000, 0.671875000000
0.000000000000, 1.000000000000, 0.695312500000
0.000000000000, 1.000000000000, 0.718750000000
0.000000000000, 1.000000000000, 0.742187500000
0.000000000000, 1.000000000000, 0.765625000000
0.000000000000, 1.000000000000, 0.789062500000
0.000000000000, 1.000000000000, 0.812500000000
0.000000000000, 1.000000000000, 0.835937500000
0.000000000000, 1.000000000000, 0.859375000000
0.000000000000, 1.000000000000, 0.882812500000
0.000000000000, 1.000000000000, 0.906250000000
0.000000000000, 1.000000000000, 0.929687500000
0.000000000000, 1.000000000000, 0.953125000000
0.000000000000, 1.000000000000, 0.976562500000
0.000000000000, 1.000000000000, 1.000000000000
0.000000000000, 0.976562500000, 1.000000000000
0.000000000000, 0.953125000000, 1.000000000000
0.000000000000, 0.929687500000, 1.000000000000
0.000000000000, 0.906250000000, 1.000000000000
0.000000000000, 0.882812500000, 1.000000000000
0.000000000000, 0.859375000000, 1.000000000000
0.000000000000, 0.835937500000, 1.000000000000
0.000000000000, 0.812500000000, 1.000000000000
0.000000000000, 0.789062500000, 1.000000000000
0.000000000000, 0.765625000000, 1.000000000000
0.000000000000, 0.742187500000, 1.000000000000
0.000000000000, 0.718750000000, 1.000000000000
0.000000000000, 0.695312500000, 1.000000000000
0.000000000000, 0.671875000000, 1.000000000000
0.000000000000, 0.648437500000, 1.000000000000
0.000000000000, 0.625000000000, 1.000000000000
0.000000000000, 0.601562500000, 1.000000000000
0.000000000000, 0.578125000000, 1.000000000000
0.000000000000, 0.554687500000, 1.000000000000
0.000000000000, 0.531250000000, 1.000000000000
0.000000000000, 0.507812500000, 1.000000000000
0.000000000000, 0.484375000000, 1.000000000000
0.000000000000, 0.460937500000, 1.000000000000
0.000000000000, 0.437500000000, 1.000000000000
0.000000000000, 0.414062500000, 1.000000000000
0.000000000000, 0.390625000000, 1.000000000000
0.000000000000, 0.367187500000, 1.000000000000
0.000000000000, 0.343750000000, 1.000000000000
0.000000000000, 0.320312500000, 1.000000000000
0.000000000000, 0.296875000000, 1.000000000000
0.000000000000, 0.273437500000, 1.000000000000
0.000000000000, 0.250000000000, 1.000000000000
0.000000000000, 0.226562500000, 1.000000000000
0.000000000000, 0.203125000000, 1.000000000000
0.000000000000, 0.179687500000, 1.000000000000
0.000000000000, 0.156250000000, 1.000000000000
0.000000000000, 0.132812500000, 1.000000000000
0.000000000000, 0.109375000000, 1.000000000000
0.000000000000, 0.085937500000, 1.000000000000
0.000000000000, 0.062500000000, 1.000000000000
0.000000000000, 0.039062500000, 1.000000000000
0.000000000000, 0.015625000000, 1.000000000000
0.007812500000, 0.000000000000, 1.000000000000
0.031250000000, 0.000000000000, 1.000000000000
0.054687500000, 0.000000000000, 1.000000000000
0.078125000000, 0.000000000000, 1.000000000000
0.101562500000, 0.000000000000, 1.000000000000
0.125000000000, 0.000000000000, 1.000000000000
0.148437500000, 0.000000000000, 1.000000000000
0.171875000000, 0.000000000000, 1.000000000000
0.195312500000, 0.000000000000, 1.000000000000
0.218750000000, 0.000000000000, 1.000000000000
0.242187500000, 0.000000000000, 1.000000000000
0.265625000000, 0.000000000000, 1.000000000000
0.289062500000, 0.000000000000, 1.000000000000
0.312500000000, 0.000000000000, 1.000000000000
0.335937500000, 0.000000000000, 1.000000000000
0.359375000000, 0.000000000000, 1.000000000000
0.382812500000, 0.000000000000, 1.000000000000
0.406250000000, 0.000000000000, 1.000000000000
0.429687500000, 0.000000000000, 1.000000000000
0.453125000000, 0.000000000000, 1.000000000000
0.476562500000, 0.000000000000, 1.000000000000
0.500000000000, 0.000000000000, 1.000000000000
0.523437500000, 0.000000000000, 1.000000000000
0.546875000000, 0.000000000000, 1.000000000000
0.570312500000, 0.000000000000, 1.000000000000
0.593750000000, 0.000000000000, 1.000000000000
0.617187500000, 0.000000000000, 1.000000000000
0.640625000000, 0.000000000000, 1.000000000000
0.664062500000, 0.000000000000, 1.000000000000
0.687500000000, 0.000000000000, 1.000000000000
0.710937500000, 0.000000000000, 1.000000000000
0.734375000000, 0.000000000000, 1.000000000000
0.757812500000, 0.000000000000, 1.000000000000
0.781250000000, 0.000000000000, 1.000000000000
0.804687500000, 0.000000000000, 1.000000000000
0.828125000000, 0.000000000000, 1.000000000000
0.851562500000, 0.000000000000, 1.000000000000
0.875000000000, 0.000000000000, 1.000000000000
0.898437500000, 0.000000000000, 1.000000000000
0.921875000000, 0.000000000000, 1.000000000000
0.945312500000, 0.000000000000, 1.000000000000
0.968750000000, 0.000000000000, 1.000000000000
0.992187500000, 0.000000000000, 1.000000000000
1.000000000000, 0.000000000000, 0.984375000000
1.000000000000, 0.000000000000, 0.960937500000
1.000000000000, 0.000000000000, 0.937500000000
1.000000000000, 0.000000000000, 0.914062500000
1.000000000000, 0.000000000000, 0.890625000000
1.000000000000, 0.000000000000, 0.867187500000
1.000000000000, 0.000000000000, 0.843750000000
1.000000000000, 0.000000000000, 0.820312500000
1.000000000000, 0.000000000000, 0.796875000000
1.000000000000, 0.000000000000, 0.773437500000
1.000000000000, 0.000000000000, 0.750000000000
1.000000000000, 0.000000000000, 0.726562500000
1.000000000000, 0.000000000000, 0.703125000000
1.000000000000, 0.000000000000, 0.679687500000
1.000000000000, 0.000000000000, 0.656250000000
1.000000000000, 0.000000000000, 0.632812500000
1.000000000000, 0.000000000000, 0.609375000000
1.000000000000, 0.000000000000, 0.585937500000
1.000000000000, 0.000000000000, 0.562500000000
1.000000000000, 0.000000000000, 0.539062500000
1.000000000000, 0.000000000000, 0.515625000000
1.000000000000, 0.000000000000, 0.492187500000
1.000000000000, 0.000000000000, 0.468750000000
1.000000000000, 0.000000000000, 0.445312500000
1.000000000000, 0.000000000000, 0.421875000000
1.000000000000, 0.000000000000, 0.398437500000
1.000000000000, 0.000000000000, 0.375000000000
1.000000000000, 0.000000000000, 0.351562500000
1.000000000000, 0.000000000000, 0.328125000000
1.000000000000, 0.000000000000, 0.304687500000
1.000000000000, 0.000000000000, 0.281250000000
1.000000000000, 0.000000000000, 0.257812500000
1.000000000000, 0.000000000000, 0.234375000000
1.000000000000, 0.000000000000, 0.210937500000
1.000000000000, 0.000000000000, 0.187500000000
1.000000000000, 0.000000000000, 0.164062500000
1.000000000000, 0.000000000000, 0.140625000000
1.000000000000, 0.000000000000, 0.117187500000
1.000000000000, 0.000000000000, 0.093750000000
1.000000000000, 0.000000000000, 0.070312500000
1.000000000000, 0.000000000000, 0.046875000000
1.000000000000, 0.000000000000, 0.023437500000];
T_end = 620.000000000000;
rader_pos = [0.000000000000
-10.000000000000
120.000000000000];
rader_Rmax = 150.000000000000;


end