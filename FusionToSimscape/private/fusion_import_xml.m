function xmlPath = fusion_import_xml(exportFolder, grounding, rootBody)
%FUSION_IMPORT_XML Prepare a temporary import without altering the CAD export.
xmlPath = string(tempname(exportFolder)) + ".xml";
document = xmlread(fullfile(exportFolder, 'assembly.xml'));
if grounding ~= "preserve"
    instances = document.getElementsByTagName('Instance');
    found = false;
    for index = 0:instances.getLength-1
        instance = instances.item(index);
        selected = string(instance.getAttribute('uid')) == rootBody;
        instance.removeAttribute('grounded');
        if selected
            instance.setAttribute('grounded', 'true');
            found = true;
        end
    end
    assert(found, 'FusionToSimscape:RootBody', 'Root body UID was not found in assembly.xml.');
end
xmlwrite(xmlPath, document);
end
