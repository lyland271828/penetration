function [pos_detected_x_ouput,pos_detected_y_ouput] = save_pos_detected(pos_detected_x_input, pos_detected_y_input)
%SAVE_DATA 

persistent values1 values2

if isempty(values1)
    values1 = [];
    values2 = [];
end

if nargin>0
    values1 = pos_detected_x_input;
    values2 = pos_detected_y_input;
end

pos_detected_x_ouput = values1;
pos_detected_y_ouput = values2;

end

