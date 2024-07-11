
function cm = arbre(n, varargin)
% Colormap: arbre

%-- Parse inputs ---------------------------------------------------------%
if ~exist('n', 'var'); n = []; end
if isempty(n)
   f = get(groot,'CurrentFigure');
   if isempty(f)
      n = size(get(groot,'DefaultFigureColormap'),1);
   else
      n = size(f.Colormap,1);
   end
end
%-------------------------------------------------------------------------%

% Data for colormap:
cm = [
	0.441318000	0.056262000	0.049513000
	0.443702000	0.060636000	0.060832000
	0.446059000	0.064862000	0.071651000
	0.448391000	0.068958000	0.082109000
	0.450675000	0.072961000	0.092473000
	0.452935000	0.076854000	0.102606000
	0.455169000	0.080649000	0.112578000
	0.457360000	0.084371000	0.122551000
	0.459530000	0.088005000	0.132385000
	0.461671000	0.091563000	0.142156000
	0.463777000	0.095058000	0.151941000
	0.465864000	0.098478000	0.161625000
	0.467920000	0.101839000	0.171317000
	0.469949000	0.105140000	0.180998000
	0.471964000	0.108376000	0.190595000
	0.473942000	0.111563000	0.200266000
	0.475905000	0.114692000	0.209880000
	0.477852000	0.117765000	0.219460000
	0.479768000	0.120791000	0.229100000
	0.481676000	0.123762000	0.238663000
	0.483559000	0.126685000	0.248273000
	0.485429000	0.129558000	0.257867000
	0.487290000	0.132381000	0.267419000
	0.489124000	0.135158000	0.277047000
	0.490958000	0.137885000	0.286602000
	0.492771000	0.140566000	0.296220000
	0.494578000	0.143199000	0.305814000
	0.496378000	0.145785000	0.315404000
	0.498162000	0.148324000	0.325042000
	0.499950000	0.150816000	0.334621000
	0.501715000	0.153261000	0.344309000
	0.503487000	0.155660000	0.353933000
	0.505243000	0.158011000	0.363638000
	0.507000000	0.160317000	0.373323000
	0.508748000	0.162575000	0.383056000
	0.510492000	0.164788000	0.392809000
	0.512232000	0.166955000	0.402589000
	0.513963000	0.169075000	0.412421000
	0.515693000	0.171152000	0.422266000
	0.517411000	0.173181000	0.432186000
	0.519128000	0.175168000	0.442115000
	0.520831000	0.177109000	0.452132000
	0.522530000	0.179010000	0.462165000
	0.524215000	0.180865000	0.472286000
	0.525892000	0.182681000	0.482440000
	0.527553000	0.184456000	0.492674000
	0.529200000	0.186193000	0.502966000
	0.530832000	0.187893000	0.513315000
	0.532442000	0.189555000	0.523760000
	0.534037000	0.191191000	0.534228000
	0.535597000	0.192786000	0.544837000
	0.537141000	0.194362000	0.555454000
	0.538646000	0.195904000	0.566204000
	0.540122000	0.197429000	0.577001000
	0.541564000	0.198938000	0.587861000
	0.542957000	0.200426000	0.598840000
	0.544313000	0.201913000	0.609843000
	0.545614000	0.203391000	0.620955000
	0.546859000	0.204871000	0.632136000
	0.548053000	0.206366000	0.643344000
	0.549172000	0.207870000	0.654662000
	0.550221000	0.209399000	0.666028000
	0.551199000	0.210967000	0.677414000
	0.552088000	0.212575000	0.688864000
	0.552882000	0.214234000	0.700366000
	0.553582000	0.215963000	0.711869000
	0.554178000	0.217772000	0.723367000
	0.554651000	0.219670000	0.734896000
	0.554999000	0.221674000	0.746413000
	0.555215000	0.223802000	0.757884000
	0.555290000	0.226071000	0.769295000
	0.555213000	0.228495000	0.780629000
	0.554965000	0.231093000	0.791885000
	0.554539000	0.233882000	0.803031000
	0.553927000	0.236882000	0.814026000
	0.553118000	0.240110000	0.824844000
	0.552100000	0.243583000	0.835455000
	0.550862000	0.247317000	0.845830000
	0.549393000	0.251326000	0.855935000
	0.547682000	0.255623000	0.865737000
	0.545720000	0.260218000	0.875201000
	0.543499000	0.265117000	0.884290000
	0.541012000	0.270325000	0.892972000
	0.538253000	0.275840000	0.901211000
	0.535220000	0.281658000	0.908978000
	0.531911000	0.287771000	0.916241000
	0.528328000	0.294166000	0.922977000
	0.524473000	0.300827000	0.929164000
	0.520352000	0.307734000	0.934784000
	0.515973000	0.314866000	0.939826000
	0.511346000	0.322195000	0.944284000
	0.506483000	0.329697000	0.948156000
	0.501396000	0.337345000	0.951446000
	0.496104000	0.345108000	0.954162000
	0.490624000	0.352960000	0.956318000
	0.484972000	0.360874000	0.957932000
	0.479168000	0.368823000	0.959024000
	0.473229000	0.376785000	0.959619000
	0.467174000	0.384737000	0.959741000
	0.461022000	0.392660000	0.959420000
	0.454791000	0.400536000	0.958683000
	0.448500000	0.408348000	0.957561000
	0.442167000	0.416083000	0.956082000
	0.435805000	0.423731000	0.954276000
	0.429430000	0.431283000	0.952170000
	0.423057000	0.438730000	0.949792000
	0.416699000	0.446068000	0.947168000
	0.410368000	0.453290000	0.944323000
	0.404077000	0.460395000	0.941282000
	0.397835000	0.467381000	0.938066000
	0.391654000	0.474245000	0.934696000
	0.385543000	0.480989000	0.931193000
	0.379509000	0.487612000	0.927575000
	0.373561000	0.494115000	0.923859000
	0.367705000	0.500502000	0.920059000
	0.361949000	0.506772000	0.916192000
	0.356296000	0.512930000	0.912269000
	0.350753000	0.518978000	0.908304000
	0.345323000	0.524919000	0.904306000
	0.340011000	0.530756000	0.900287000
	0.334819000	0.536492000	0.896255000
	0.329751000	0.542132000	0.892219000
	0.324807000	0.547678000	0.888185000
	0.319989000	0.553134000	0.884162000
	0.315299000	0.558504000	0.880153000
	0.310736000	0.563791000	0.876166000
	0.306301000	0.569000000	0.872203000
	0.301992000	0.574132000	0.868271000
	0.297809000	0.579193000	0.864371000
	0.293749000	0.584185000	0.860506000
	0.289811000	0.589111000	0.856680000
	0.285991000	0.593976000	0.852895000
	0.282288000	0.598782000	0.849152000
	0.278696000	0.603532000	0.845452000
	0.275212000	0.608231000	0.841797000
	0.271830000	0.612880000	0.838185000
	0.268546000	0.617482000	0.834618000
	0.265355000	0.622042000	0.831094000
	0.262250000	0.626560000	0.827614000
	0.259225000	0.631041000	0.824176000
	0.256275000	0.635487000	0.820780000
	0.253393000	0.639899000	0.817424000
	0.250571000	0.644282000	0.814106000
	0.247804000	0.648636000	0.810824000
	0.245084000	0.652965000	0.807576000
	0.242405000	0.657270000	0.804360000
	0.239758000	0.661553000	0.801173000
	0.237137000	0.665818000	0.798011000
	0.234536000	0.670064000	0.794873000
	0.231947000	0.674295000	0.791753000
	0.229364000	0.678511000	0.788650000
	0.226782000	0.682715000	0.785560000
	0.224194000	0.686908000	0.782478000
	0.221596000	0.691091000	0.779400000
	0.218982000	0.695265000	0.776324000
	0.216348000	0.699432000	0.773243000
	0.213691000	0.703593000	0.770154000
	0.211008000	0.707748000	0.767052000
	0.208297000	0.711898000	0.763933000
	0.205557000	0.716045000	0.760792000
	0.202789000	0.720188000	0.757624000
	0.199995000	0.724328000	0.754425000
	0.197176000	0.728465000	0.751189000
	0.194338000	0.732600000	0.747913000
	0.191486000	0.736733000	0.744591000
	0.188628000	0.740863000	0.741218000
	0.185773000	0.744991000	0.737790000
	0.182933000	0.749116000	0.734302000
	0.180122000	0.753238000	0.730748000
	0.177356000	0.757357000	0.727125000
	0.174655000	0.761471000	0.723427000
	0.172039000	0.765581000	0.719651000
	0.169535000	0.769685000	0.715791000
	0.167167000	0.773783000	0.711842000
	0.164969000	0.777873000	0.707802000
	0.162973000	0.781955000	0.703665000
	0.161216000	0.786027000	0.699428000
	0.159735000	0.790089000	0.695087000
	0.158571000	0.794138000	0.690639000
	0.157765000	0.798174000	0.686079000
	0.157358000	0.802195000	0.681405000
	0.157389000	0.806199000	0.676613000
	0.157896000	0.810185000	0.671700000
	0.158915000	0.814151000	0.666664000
	0.160475000	0.818095000	0.661501000
	0.162601000	0.822016000	0.656209000
	0.165313000	0.825912000	0.650785000
	0.168622000	0.829780000	0.645229000
	0.172534000	0.833620000	0.639536000
	0.177049000	0.837428000	0.633704000
	0.182159000	0.841203000	0.627733000
	0.187854000	0.844943000	0.621622000
	0.194117000	0.848646000	0.615369000
	0.200929000	0.852309000	0.608973000
	0.208267000	0.855931000	0.602435000
	0.216108000	0.859508000	0.595752000
	0.224427000	0.863040000	0.588925000
	0.233198000	0.866524000	0.581954000
	0.242398000	0.869958000	0.574839000
	0.252002000	0.873339000	0.567580000
	0.261991000	0.876665000	0.560169000
	0.272339000	0.879934000	0.552615000
	0.283025000	0.883145000	0.544918000
	0.294028000	0.886293000	0.537081000
	0.305331000	0.889379000	0.529104000
	0.316914000	0.892399000	0.520990000
	0.328770000	0.895351000	0.512727000
	0.340877000	0.898233000	0.504329000
	0.353218000	0.901043000	0.495799000
	0.365779000	0.903780000	0.487140000
	0.378553000	0.906440000	0.478348000
	0.391530000	0.909023000	0.469423000
	0.404690000	0.911526000	0.460377000
	0.418022000	0.913949000	0.451216000
	0.431526000	0.916287000	0.441928000
	0.445185000	0.918541000	0.432526000
	0.458984000	0.920710000	0.423021000
	0.472923000	0.922791000	0.413407000
	0.486991000	0.924783000	0.403688000
	0.501170000	0.926686000	0.393884000
	0.515464000	0.928497000	0.383982000
	0.529856000	0.930216000	0.373999000
	0.544333000	0.931845000	0.363949000
	0.558903000	0.933378000	0.353818000
	0.573535000	0.934820000	0.343642000
	0.588243000	0.936166000	0.333406000
	0.602997000	0.937421000	0.323144000
	0.617805000	0.938581000	0.312851000
	0.632643000	0.939649000	0.302559000
	0.647518000	0.940624000	0.292268000
	0.662403000	0.941509000	0.282021000
	0.677310000	0.942302000	0.271817000
	0.692206000	0.943008000	0.261711000
	0.707100000	0.943625000	0.251720000
	0.721973000	0.944157000	0.241896000
	0.736808000	0.944609000	0.232298000
	0.751613000	0.944977000	0.222968000
	0.766363000	0.945270000	0.213999000
	0.781043000	0.945490000	0.205482000
	0.795643000	0.945641000	0.197524000
	0.810161000	0.945725000	0.190242000
	0.824569000	0.945750000	0.183800000
	0.838850000	0.945722000	0.178365000
	0.852986000	0.945649000	0.174126000
	0.866956000	0.945538000	0.171281000
	0.880732000	0.945400000	0.170031000
	0.894283000	0.945248000	0.170568000
	0.907569000	0.945096000	0.173059000
	0.920541000	0.944966000	0.177640000
	0.933148000	0.944876000	0.184411000
	0.945305000	0.944863000	0.193434000
	0.956906000	0.944969000	0.204736000
	0.967834000	0.945242000	0.218337000
	0.977910000	0.945757000	0.234198000
	0.986926000	0.946604000	0.252236000
	0.994649000	0.947887000	0.272268000
];

% Modify the colormap by interpolation to match number of waypoints.
cm = tools.interpolate(cm, n, varargin{:});

end