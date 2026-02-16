<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="25008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="Menus" Type="Folder" URL="../Menus">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="Typedef" Type="Folder" URL="../Typedef">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="DistanceSensorDriver.lvlib" Type="Library" URL="../DistanceSensorDriver.lvlib"/>
		<Item Name="Dependencies" Type="Dependencies"/>
		<Item Name="Build Specifications" Type="Build">
			<Item Name="2m Distance Sensor Package" Type="{E661DAE2-7517-431F-AC41-30807A3BDA38}">
				<Property Name="NIPKG_addToFeed" Type="Bool">true</Property>
				<Property Name="NIPKG_allDependenciesToFeed" Type="Bool">false</Property>
				<Property Name="NIPKG_allDependenciesToSystemLink" Type="Bool">false</Property>
				<Property Name="NIPKG_certificates" Type="Bool">false</Property>
				<Property Name="NIPKG_createInstaller" Type="Bool">true</Property>
				<Property Name="NIPKG_feedLocation" Type="Path">/C/Users/Public/Documents/REV_NIPM</Property>
				<Property Name="NIPKG_installerArtifacts" Type="Str">Install.exe|InstallCHS.dll|InstallDEU.dll|InstallFRA.dll|InstallJPN.dll|InstallKOR.dll|bin|feeds|pool|system-packages
</Property>
				<Property Name="NIPKG_installerBuiltBefore" Type="Bool">true</Property>
				<Property Name="NIPKG_installerDestination" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/builds/Installer</Property>
				<Property Name="NIPKG_lastBuiltPackage" Type="Str">2m-distance-sensor_2026.0.0-0_windows_all.nipkg</Property>
				<Property Name="NIPKG_license" Type="Ref"></Property>
				<Property Name="NIPKG_packageVersion" Type="Bool">false</Property>
				<Property Name="NIPKG_releaseNotes" Type="Str"></Property>
				<Property Name="NIPKG_storeProduct" Type="Bool">false</Property>
				<Property Name="NIPKG_VisibleForRuntimeDeployment" Type="Bool">false</Property>
				<Property Name="PKG_actions.Count" Type="Int">0</Property>
				<Property Name="PKG_autoIncrementBuild" Type="Bool">false</Property>
				<Property Name="PKG_autoSelectDeps" Type="Bool">true</Property>
				<Property Name="PKG_buildNumber" Type="Int">0</Property>
				<Property Name="PKG_buildSpecName" Type="Str">2m Distance Sensor Package</Property>
				<Property Name="PKG_dependencies.Count" Type="Int">1</Property>
				<Property Name="PKG_dependencies[0].Enhanced" Type="Bool">false</Property>
				<Property Name="PKG_dependencies[0].MaxVersion" Type="Str"></Property>
				<Property Name="PKG_dependencies[0].MaxVersionInclusive" Type="Bool">false</Property>
				<Property Name="PKG_dependencies[0].MinVersion" Type="Str">25.3.3.49167-0+f15</Property>
				<Property Name="PKG_dependencies[0].MinVersionType" Type="Str">Inclusive</Property>
				<Property Name="PKG_dependencies[0].NIPKG.DisplayName" Type="Str">LabVIEW Runtime (32-bit)</Property>
				<Property Name="PKG_dependencies[0].Package.Name" Type="Str">ni-labview-2025-runtime-engine-x86</Property>
				<Property Name="PKG_dependencies[0].Package.Section" Type="Str">Programming Environments</Property>
				<Property Name="PKG_dependencies[0].Package.Synopsis" Type="Str">The LabVIEW Runtime is a software add-on that enables engineers to run executables on a nondevelopment machine.</Property>
				<Property Name="PKG_dependencies[0].Relationship" Type="Str">Required Dependency</Property>
				<Property Name="PKG_dependencies[0].Type" Type="Str">NIPKG</Property>
				<Property Name="PKG_description" Type="Str">LabVIEW files for REV 2m Distance sensor for use in the FIRST Robotics Competition</Property>
				<Property Name="PKG_destinations.Count" Type="Int">6</Property>
				<Property Name="PKG_destinations[0].ID" Type="Str">{1D77AFBE-C7CE-459C-AE8F-6010E2DA1287}</Property>
				<Property Name="PKG_destinations[0].Subdir.Directory" Type="Str">WPI</Property>
				<Property Name="PKG_destinations[0].Subdir.Parent" Type="Str">{ADBD6C0A-56EF-41E1-8A7D-59D0EFFA4258}</Property>
				<Property Name="PKG_destinations[0].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[1].ID" Type="Str">{38A4886A-36D8-4C1E-B228-E1694D7B510D}</Property>
				<Property Name="PKG_destinations[1].Subdir.Directory" Type="Str">National Instruments</Property>
				<Property Name="PKG_destinations[1].Subdir.Parent" Type="Str">root_5</Property>
				<Property Name="PKG_destinations[1].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[2].ID" Type="Str">{ADBD6C0A-56EF-41E1-8A7D-59D0EFFA4258}</Property>
				<Property Name="PKG_destinations[2].Subdir.Directory" Type="Str">Rock Robotics</Property>
				<Property Name="PKG_destinations[2].Subdir.Parent" Type="Str">{E8631500-A6BD-42B6-9A39-D108C2D73D05}</Property>
				<Property Name="PKG_destinations[2].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[3].ID" Type="Str">{CA59B132-AD2C-48FA-8499-0AEBC3D9C468}</Property>
				<Property Name="PKG_destinations[3].Subdir.Directory" Type="Str">ThirdParty</Property>
				<Property Name="PKG_destinations[3].Subdir.Parent" Type="Str">{1D77AFBE-C7CE-459C-AE8F-6010E2DA1287}</Property>
				<Property Name="PKG_destinations[3].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[4].ID" Type="Str">{E8631500-A6BD-42B6-9A39-D108C2D73D05}</Property>
				<Property Name="PKG_destinations[4].Subdir.Directory" Type="Str">vi.lib</Property>
				<Property Name="PKG_destinations[4].Subdir.Parent" Type="Str">{FF584A78-43CB-4A2A-9DB3-1A0A38B1237E}</Property>
				<Property Name="PKG_destinations[4].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[5].ID" Type="Str">{FF584A78-43CB-4A2A-9DB3-1A0A38B1237E}</Property>
				<Property Name="PKG_destinations[5].Subdir.Directory" Type="Str">LabVIEW 2025</Property>
				<Property Name="PKG_destinations[5].Subdir.Parent" Type="Str">{38A4886A-36D8-4C1E-B228-E1694D7B510D}</Property>
				<Property Name="PKG_destinations[5].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_displayName" Type="Str">REV 2m Distance Sensor</Property>
				<Property Name="PKG_displayVersion" Type="Str"></Property>
				<Property Name="PKG_feedDescription" Type="Str"></Property>
				<Property Name="PKG_feedName" Type="Str"></Property>
				<Property Name="PKG_homepage" Type="Str">www.revrobotics.com</Property>
				<Property Name="PKG_hostname" Type="Str"></Property>
				<Property Name="PKG_maintainer" Type="Str">REV Robotics &lt;support@revrobotics.com&gt;</Property>
				<Property Name="PKG_output" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/builds</Property>
				<Property Name="PKG_packageName" Type="Str">2m-distance-sensor</Property>
				<Property Name="PKG_publishToSystemLink" Type="Bool">false</Property>
				<Property Name="PKG_section" Type="Str">Add-Ons</Property>
				<Property Name="PKG_shortcuts.Count" Type="Int">0</Property>
				<Property Name="PKG_sources.Count" Type="Int">1</Property>
				<Property Name="PKG_sources[0].Destination" Type="Str">{CA59B132-AD2C-48FA-8499-0AEBC3D9C468}</Property>
				<Property Name="PKG_sources[0].ID" Type="Ref">/My Computer/Build Specifications/WPILib Third-Party</Property>
				<Property Name="PKG_sources[0].Type" Type="Str">Build</Property>
				<Property Name="PKG_synopsis" Type="Str">REV 2m Distance Sensor LabVIEW API</Property>
				<Property Name="PKG_version" Type="Str">2026.0.0</Property>
			</Item>
			<Item Name="WPILib Third-Party" Type="Source Distribution">
				<Property Name="Bld_buildCacheID" Type="Str">{7C52C9E8-1805-4972-9778-4F7F5AEE00E3}</Property>
				<Property Name="Bld_buildSpecName" Type="Str">WPILib Third-Party</Property>
				<Property Name="Bld_excludedDirectory[0]" Type="Path">vi.lib</Property>
				<Property Name="Bld_excludedDirectory[0].pathType" Type="Str">relativeToAppDir</Property>
				<Property Name="Bld_excludedDirectory[1]" Type="Path">resource/objmgr</Property>
				<Property Name="Bld_excludedDirectory[1].pathType" Type="Str">relativeToAppDir</Property>
				<Property Name="Bld_excludedDirectory[2]" Type="Path">/C/ProgramData/National Instruments/InstCache/25.0</Property>
				<Property Name="Bld_excludedDirectory[3]" Type="Path">/C/Users/Jan-Felix Abellera/Documents/LabVIEW Data/2025(32-bit)/ExtraVILib</Property>
				<Property Name="Bld_excludedDirectory[4]" Type="Path">instr.lib</Property>
				<Property Name="Bld_excludedDirectory[4].pathType" Type="Str">relativeToAppDir</Property>
				<Property Name="Bld_excludedDirectory[5]" Type="Path">user.lib</Property>
				<Property Name="Bld_excludedDirectory[5].pathType" Type="Str">relativeToAppDir</Property>
				<Property Name="Bld_excludedDirectoryCount" Type="Int">6</Property>
				<Property Name="Bld_localDestDir" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/WPILib/ThirdParty</Property>
				<Property Name="Bld_previewCacheID" Type="Str">{BD7521DF-B566-4760-9656-AE8025F6DCEB}</Property>
				<Property Name="Bld_removeVIObj" Type="Int">1</Property>
				<Property Name="Bld_version.major" Type="Int">2026</Property>
				<Property Name="Destination[0].destName" Type="Str">Destination Directory</Property>
				<Property Name="Destination[0].path" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/WPILib/ThirdParty</Property>
				<Property Name="Destination[0].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[1].destName" Type="Str">Support Directory</Property>
				<Property Name="Destination[1].path" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/WPILib/ThirdParty/data</Property>
				<Property Name="Destination[1].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[2].destName" Type="Str">Private</Property>
				<Property Name="Destination[2].path" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/SubVI/Private</Property>
				<Property Name="Destination[2].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[3].destName" Type="Str">Public</Property>
				<Property Name="Destination[3].path" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/SubVI/Public</Property>
				<Property Name="Destination[3].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[4].destName" Type="Str">Typedef</Property>
				<Property Name="Destination[4].path" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/Typedef</Property>
				<Property Name="Destination[4].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[5].destName" Type="Str">Menu</Property>
				<Property Name="Destination[5].path" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/WPILib/ThirdParty</Property>
				<Property Name="Destination[5].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[5].preserveHierarchy" Type="Bool">true</Property>
				<Property Name="Destination[6].destName" Type="Str">REV Root</Property>
				<Property Name="Destination[6].path" Type="Path">/C/Users/Public/Documents/REV2m-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME</Property>
				<Property Name="Destination[6].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="DestinationCount" Type="Int">7</Property>
				<Property Name="Source[0].itemID" Type="Str">{0986E4C0-19BE-4A6D-B1F0-F2301E8EE982}</Property>
				<Property Name="Source[0].type" Type="Str">Container</Property>
				<Property Name="Source[1].Container.applyDestination" Type="Bool">true</Property>
				<Property Name="Source[1].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[1].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[1].destinationIndex" Type="Int">5</Property>
				<Property Name="Source[1].itemID" Type="Ref">/My Computer/Menus</Property>
				<Property Name="Source[1].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[1].type" Type="Str">Container</Property>
				<Property Name="Source[2].Container.applyDestination" Type="Bool">true</Property>
				<Property Name="Source[2].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[2].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[2].destinationIndex" Type="Int">4</Property>
				<Property Name="Source[2].itemID" Type="Ref">/My Computer/Typedef</Property>
				<Property Name="Source[2].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[2].type" Type="Str">Container</Property>
				<Property Name="Source[3].destinationIndex" Type="Int">6</Property>
				<Property Name="Source[3].itemID" Type="Ref">/My Computer/DistanceSensorDriver.lvlib</Property>
				<Property Name="Source[3].Library.allowMissingMembers" Type="Bool">true</Property>
				<Property Name="Source[3].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[3].type" Type="Str">Library</Property>
				<Property Name="Source[4].Container.applyDestination" Type="Bool">true</Property>
				<Property Name="Source[4].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[4].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[4].destinationIndex" Type="Int">2</Property>
				<Property Name="Source[4].itemID" Type="Ref">/My Computer/DistanceSensorDriver.lvlib/Private</Property>
				<Property Name="Source[4].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[4].type" Type="Str">Container</Property>
				<Property Name="Source[5].Container.applyDestination" Type="Bool">true</Property>
				<Property Name="Source[5].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[5].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[5].destinationIndex" Type="Int">3</Property>
				<Property Name="Source[5].itemID" Type="Ref">/My Computer/DistanceSensorDriver.lvlib/Public</Property>
				<Property Name="Source[5].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[5].type" Type="Str">Container</Property>
				<Property Name="Source[6].destinationIndex" Type="Int">6</Property>
				<Property Name="Source[6].itemID" Type="Ref">/My Computer/DistanceSensorDriver.lvlib/libDistanceSensorDriver.so</Property>
				<Property Name="Source[6].sourceInclusion" Type="Str">Include</Property>
				<Property Name="SourceCount" Type="Int">7</Property>
			</Item>
		</Item>
	</Item>
</Project>
