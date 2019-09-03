function [statenames, stateblocks] = getstatenames(mdl)
%% GETSTATENAMES
%
% This function will identify the states of a given simulink model.  A mapping
% to blockname will be assumed for this example.  It is up to the user to map
% states as they would wish or simply to use Simulink's definition of full block
% path if they would like.
%
% Simulink uses full block path as a manner to assure uniqueness, allow
% for scalability, and always have clear meaning.  It is realized that
% long block paths can result from large, highly nested models,
% yet if automated scripting is used for manipulation, then
% this is seen as an effective, consistant, and error resistant approach.
% Especially when combined with a user defined mapping to things that are
% shorter and can comply with company model styleguides and conventions.
%
% This file will additionally remove any new line characters in the
% statename and blockpath.

% Copyright 2004 - 2010 The MathWorks, Inc.

%% Extract State Information
% extract the state information (see <http://www.mathworks.com/access/helpdesk/help/toolbox/simulink/slref/model_cmd.html MODEL command>
% for more info)
% This is one main command that can be leveraged by users in their own
% scripts.  If you walk away with anything remember this one, and bookmark
% the above page in your help browsers.
%
% It is important to realize that Simulink uses complete block path
% to identify unique states within a Simulink model.  It is the stateblocks
% variable that is returned by the following command the shows what
% Simulink sees as the continuous and discrete states in a model.
[sys,x0,stateblocks]=feval(mdl,[],[],[],0);

%% Example of State Mapping to Block Name
% Here is one example of user defined state mapping.
% Let's extract the block name from the handle to the block
% This assumes you name the block what you would name the state.  Also note
% that this may not be unique (ie blocks with state within libraries will all
% return the same thing for each instance).
statenames = get_param(stateblocks,'Name');

%% Remove New Linesn within names
% lets remove the newline characters for ease on the eyes
statenames = regexprep(statenames,'\n',' ');
stateblocks = regexprep(stateblocks,'\n',' ');

