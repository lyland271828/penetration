function values =evaluation_rader_project_module_average(time_series, values_series)
%EVALUATION_RADER_PROJECT_MODULE_AVERAGE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_average.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by evaluation_rader_project_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'evaluation_rader_project_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func(strcat(file_name_param,str_core));
end
[v_flock,...
r_coll,...
target] = fun_params();

%
phiCorr = mean(values_series(1,:));
phiVel  = mean(values_series(2,:));
phiColl = mean(values_series(3,:));
phiWall = mean(values_series(4,:));
phigroup = mean(values_series(5,:));

% [~,~,~,~,F] = fitness_function_combine(v_flock,a_tol,v_tol,r_tol,phiVel, ...
%     phiColl,phiWall,phiCorr);

values = [phiCorr;phiVel;phiColl;phiWall;phigroup];





end