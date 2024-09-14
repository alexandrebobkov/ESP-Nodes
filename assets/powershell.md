# Working with Files and Folders

## Get-Help

## Get-Children

Gets items in a specified location. To list the folders in my drive C, I will run
the command below:

```
Get-ChildItem c:\
```

This will list all the top-level folders. To list all files, folders include sub-folders use the `-Recurse` parameter.

## Copy-Item and Move-Item

You could use the __Get-ChildItem__ Cmdlet to list items in a folder, then pipe
the result to __Copy-Item__ Cmdlet to copy the items to a new location. The
command below will do the job:

```
Get-ChildItem C:\Dropbox | Copy-Item -Destination C:\NewFolder
```

The above PowerShell command will only copy the top-level folders and
files - it will NOT copy sub-folders and files. To copy all files and folders
including sub-folders, include the -Recurse parameter in the __Get-ChildItem__
command as shown below:

```
Get-ChildItem C:\Dropbox -Recurse | Copy-Item -Destination C:\NewFolder
```

While the __Copy-Item__ Cmdlet copies items from one location to another the
__Move-Item__ Cmdlet moves the item.

## RemoveItem

## NewItem

__New-Item__ can be used to create files, folders and registry keys and entries. The command below creates a text
file called weekly_file.txt in c:\logfiles folder:
```
New-Item -Path c:\logfiles -Name weekly_file.tx
```

## RenameItem

__Rename-Item__ Cmdlet is used to rename things in Windows. This Cmdlet can
rename files, folders and registry keys. This command will rename
weekly_file.txt to monthly_file.txt

```
Rename-Item -Path C:\logfiles\weekly_file.txt -NewName monthly_file.txt
```

## Export-Csv

__Export-Csv__ converts a set of string into CSV and saves in a file. This Cmdlet
is very important in reporting.

```
Get-Command -Verb Export
```

```
Get-Command -Verb Export | Select-Object CommandType, Name, Version, Source | Export-Csv -
NoTypeInformation -Path C:\NewFolder\ExportCommands.CSV
```

# Managing Processes

## Get-Process

This PowerShell Cmdlet lists all the processes running on a local computer. If
you use the ComputerName parameter, you can display the processes on a
remote computer.

### Start-Process and Stop-Process

The __Start-Process__ Cmdlet can start a stopped process while the __Stop-Process__ Cmdlet
can stop a running process.
To start a process, pipe the output of __Get-Process__ command to the __Start-Process__ command.
As an example, to stop a process with ID 10500, use the command below.

```
Get-Process -Id 10500 | Stop-Process
```

# Getting Computer Information

## Get-WmiObject

__Get-WmiObject__ has a parameter called -Class this allows you to specify the WMI object you wish to access. The command below will get a list of WMI
classes, Get-WmiObject -List -Class Win32*


Once you know the name of the WMI class, you can execute __Get-WmiObject__ to return useful information from a local or remote computer. Below is a list
of the most important WMI classes you may need:
- Win32_PhysicalMemory - information about available memory
- Win32_Processor - Processor information
- Win32_LogicalDisk - Logical disk drive information
- Win32_DiskDrive - Physical disk information
- Win32_OperatingSystem - Information about the operating system

To get information about the operating system, run the command below:

```
Get-WmiObject -Class Win32_OperatingSystem
```

## SYSTEMINFO

__SYSTEMINFO__ displays operating system configuration information for a local or remote computer.

```
SYSTEMINFO /FO LIST
```

## DRIVEQUERY

__ DRIVERQUERY__ displayS a list of installed device drivers on a local or remote computer.

```
DRIVEQUERY /FO TABLE
```









