function block = fusion_library_block(pattern)
%FUSION_LIBRARY_BLOCK Resolve library labels independently of embedded newlines.
load_system('sm_lib');
blocks = find_system('sm_lib', 'SearchDepth', 2, 'RegExp', 'on', 'Name', ['^' pattern '$']);
assert(isscalar(blocks), 'FusionToSimscape:LibraryBlock', ...
    'Expected one library block for %s.', pattern);
block = blocks{1};
end
