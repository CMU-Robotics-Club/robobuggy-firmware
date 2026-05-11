-------------------------------------------------------------------------------
-- TBS Agent Lite 0.95
-- release date: 2021-11
-- author: JimB40
-------------------------------------------------------------------------------
local toolName = "TNS|TBS Agent Lite|TNE"
local SP = '/SCRIPTS/TOOLS/TBSAGENTLITE/'
return {run=(loadScript(SP..'loader','Tx')(SP)).run}
