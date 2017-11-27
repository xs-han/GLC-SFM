classdef ArrayIndexClass < handle
    %ARRAYINDEXCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        array;
    end
    
    methods
        
        function this = ArrayIndexClass(init, type)
        %
        %
        
            switch nargin 
                case 0
                    this.array = [];
                case 1
                    this.array = zeros(init);
                otherwise
                    this.array = zeros(init, type);
            end
            
        end
        
        function le = length(this)
            le = length(this.array);
        end
        
        function le = isempty(this)
            le = isempty(this.array);
        end
        
        function le = size(this, dim)
            if nargin > 1
                le = size(this.array, dim);    
            else
                le = size(this.array);    
            end
        end
        
        function cind = convertIndex( this, ind)%#ok
            cind = ind+1;
        end
        
        function S = convertSubs( this, S)
            S.subs{1} = this.convertIndex( S.subs{1} );
        end
        
        function varargout = subsref(this, S) 
        %overwritten function to allow subassgn using brakets    

            if S(1).type == '.'
                if nargout < 1
                    builtin('subsref',this,S);
                else
                    str = ArrayIndexClass.crVarout(nargout);
                    eval([ '[ ' str '] = builtin(''subsref'',this,S); ']);
                end
            elseif strcmp( S(1).type , '()' )
                %S(1).subs = this.parseSubs( S(1).subs );
                
                % will convert index
                S = this.convertSubs( S );
                
                varargout = subsref(this.array, S);   
                if ~iscell(varargout)
                    varargout = {varargout};
                end
            else
                error(' for S type S(1).type either "." or "()"');
            end
            
        end

        function this = subsasgn(this, S, B) 
        %overwritten function to allow subassgn using brakets
            

                if S(1).type == '.'
                    builtin('subsasgn',this,S, B);
                elseif strcmp( S(1).type , '()' )
                    if ~isa(this, 'ArrayIndexClass')
                        this = builtin('subsasgn',this,S, B);
                    else
                        if isa( B, 'ArrayIndexClass')
                            B = B.array;
                        end
                        % will convert index
                        S = this.convertSubs( S );
                        this.array = subsasgn(this.array, S, B);
                    end
                else
                    error('WRONG input for S type S(1).type either "." or "()"' );
                end
%             end
        end
        
        
    end
    
    methods(Static = true)
       
        function str = crVarout(numb)
            
            init   =  zeros(1, 13*numb*2) + 32;
            str    =  char(init);           
            
            count = 1;
            for i = 1 : numb
                
                varstr =  ['varargout{' num2str(i) '}, '];
                ll = length(varstr);
                str( count : count+ll-1 ) = varstr;
                count = count + ll;
                
            end
            
            str = str(1:count-3);
            
        end
        
    end
    
end

