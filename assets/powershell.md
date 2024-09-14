## Get-Help

## Get-Children

Gets items in a specified location. To list the folders in my drive C, I will run
the command below:

`Get-ChildItem c:\`

This will list all the top-level folders. To list all files, folders include sub-folders use the `-Recurse` parameter.

## Copy-Item and Move-Item

You could use the __Get-ChildItem__ Cmdlet to list items in a folder, then pipe
the result to __Copy-Item__ Cmdlet to copy the items to a new location. The
command below will do the job:

`Get-ChildItem C:\Dropbox | Copy-Item -Destination C:\NewFolder`

The above PowerShell command will only copy the top-level folders and
files - it will NOT copy sub-folders and files. To copy all files and folders
including sub-folders, include the -Recurse parameter in the __Get-ChildItem__
command as shown below:

`Get-ChildItem C:\Dropbox -Recurse | Copy-Item -Destination C:\NewFolder`

While the __Copy-Item__ Cmdlet copies items from one location to another the
__Move-Item__ Cmdlet moves the item.

## RemoveItem

## NewItem

__New-Item__ can be used to create files, folders and registry keys and entries. The command below creates a text
file called weekly_file.txt in c:\logfiles folder:
`New-Item -Path c:\logfiles -Name weekly_file.tx`

## RenameItem

__Rename-Item__ Cmdlet is used to rename things in Windows. This Cmdlet can
rename files, folders and registry keys. This command will rename
weekly_file.txt to monthly_file.txt

`Rename-Item -Path C:\logfiles\weekly_file.txt -NewName monthly_file.txt`

## Export-Csv

__Export-Csv__ converts a set of string into CSV and saves in a file. This Cmdlet
is very important in reporting.

```Get-Command -Verb Export````

`Get-Command -Verb Export | Select-Object CommandType, Name, Version, Source | Export-Csv -
NoTypeInformation -Path C:\NewFolder\ExportCommands.CSV`







