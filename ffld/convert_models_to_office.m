function convert_models_to_office( )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

a = load('office/n03797390_final.mat');
model = five2four(a.model);
convertmodel(model, 'outoffice/n03797390_final')
a = load('office/n04380533_final.mat');
model = five2four(a.model);
convertmodel(model, 'outoffice/n04380533_final')


end

