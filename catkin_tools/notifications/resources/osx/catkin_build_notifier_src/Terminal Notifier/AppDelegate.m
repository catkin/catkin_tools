#import "AppDelegate.h"
#import <ScriptingBridge/ScriptingBridge.h>
#import <objc/runtime.h>

NSString * const NotificationCenterUIBundleID = @"com.apple.notificationcenterui";


@implementation NSUserDefaults (SubscriptAndUnescape)
- (id)objectForKeyedSubscript:(id)key;
{
  id obj = [self objectForKey:key];
  if ([obj isKindOfClass:[NSString class]] && [(NSString *)obj hasPrefix:@"\\"]) {
    obj = [(NSString *)obj substringFromIndex:1];
  }
  return obj;
}
@end

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)notification;
{
  NSUserNotification *userNotification = notification.userInfo[NSApplicationLaunchUserNotificationKey];
  if (userNotification) {
    [self userActivatedNotification:userNotification];
  } else {
    NSArray *runningProcesses = [[[NSWorkspace sharedWorkspace] runningApplications] valueForKey:@"bundleIdentifier"];
    if ([runningProcesses indexOfObject:NotificationCenterUIBundleID] == NSNotFound) {
      NSLog(@"[!] Unable to post a notification for the current user (%@), as it has no running NotificationCenter instance.", NSUserName());
      exit(1);
    }

    NSArray *arguments = [[NSProcessInfo processInfo] arguments];
    
    NSString *title = @"No title";
    if ([arguments count] > 1)
    {
        title = arguments[1];
    }
    
    NSString *message = @"No message";
    if ([arguments count] > 2)
    {
        message = arguments[2];
    }
    
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    NSMutableDictionary *options = [NSMutableDictionary dictionary];
    if (defaults[@"activate"]) options[@"bundleID"]         = defaults[@"activate"];

    NSUserNotification *userNotification = [NSUserNotification new];
    userNotification.title = title;
//    userNotification.subtitle = message;
    userNotification.informativeText = message;
    userNotification.userInfo = options;
    
    NSUserNotificationCenter *center = [NSUserNotificationCenter defaultUserNotificationCenter];
    center.delegate = self;
    [center scheduleNotification:userNotification];
  }
}

- (BOOL)userNotificationCenter:(NSUserNotificationCenter *)center
     shouldPresentNotification:(NSUserNotification *)userNotification;
{
  return YES;
}

// Once the notification is delivered we can exit.
- (void)userNotificationCenter:(NSUserNotificationCenter *)center
        didDeliverNotification:(NSUserNotification *)userNotification;
{
  exit(0);
}

- (void)userActivatedNotification:(NSUserNotification *)userNotification;
{
  [[NSUserNotificationCenter defaultUserNotificationCenter] removeDeliveredNotification:userNotification];

  NSString *bundleID = userNotification.userInfo[@"bundleID"];

  BOOL success = YES;
  if (bundleID) success &= [self activateAppWithBundleID:bundleID];

  exit(success ? 0 : 1);
}

- (BOOL)activateAppWithBundleID:(NSString *)bundleID;
{
  id app = [SBApplication applicationWithBundleIdentifier:bundleID];
  if (app) {
    [app activate];
    return YES;
  } else {
    NSLog(@"Unable to find an application with the specified bundle indentifier.");
    return NO;
  }
}

@end
