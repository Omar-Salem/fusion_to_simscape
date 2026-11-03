function block = fusion_library_instance(model, pattern)
%FUSION_LIBRARY_INSTANCE Find a top-level imported block through its library reference.
blocks=find_system(model,'SearchDepth',1,'ReferenceBlock',fusion_library_block(pattern));
assert(isscalar(blocks),'FusionToSimscape:ImportStructure','Expected one %s block.',pattern);
block=string(blocks{1});
end
